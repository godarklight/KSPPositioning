using System;
using System.Collections.Generic;
using System.IO;
using UnityEngine;
using MessageStream2;
namespace KSPPositioning
{
    [KSPAddon(KSPAddon.Startup.Flight, false)]
    public class KSPPositioning : MonoBehaviour
    {
        private Rect windowRect = new Rect(100, 100, 300, 300);
        private Rect window2Rect = new Rect(100, 500, 300, 300);
        private Rect dragRect = new Rect(0, 0, 10000, 20);
        private bool playbackStart = false;
        private bool playback = false;
        private bool recording = false;
        private int lastFreezeFrame = -1;
        private int freezeFrame = -1;
        private bool showInterpolate = false;
        private List<OrbitalData> pathData = new List<OrbitalData>();
        private ScreenMessage sm = null;
        private int playPos = 0;
        private double posError = 0;
        private double predictVectorSpace = 0;
        private double predictOrbitalComponents = 0;

        public void Update()
        {
            if (playback)
            {
                if (sm != null)
                {
                    ScreenMessages.RemoveMessage(sm);
                }
                sm = ScreenMessages.PostScreenMessage("Playback: " + playPos + " / " + pathData.Count, float.MaxValue, ScreenMessageStyle.UPPER_CENTER);
            }
            else
            {
                if (sm != null)
                {
                    ScreenMessages.RemoveMessage(sm);
                    sm = null;
                }
            }
        }

        public void FixedUpdate()
        {
            if (FlightGlobals.ready && FlightGlobals.ActiveVessel != null)
            {
                Vessel ourVessel = FlightGlobals.ActiveVessel;
                if (recording)
                {
                    OrbitalData newEntry = new OrbitalData(ourVessel);
                    pathData.Add(newEntry);
                }
                if (playback)
                {
                    if (freezeFrame == -1)
                    {
                        if (playbackStart)
                        {
                            playbackStart = false;
                            ourVessel.GoOnRails();
                            Planetarium.SetUniversalTime(pathData[0].t - 5f);
                        }
                        if (pathData[playPos].t < Planetarium.GetUniversalTime())
                        {
                            if (playPos % 10 == 0)
                            {
                                ApplyOrbit(pathData[playPos]);
                            }
                            else
                            {
                                posError = CompareOrbit();
                                predictVectorSpace = ExtrapolateOrbitVectorSpace();
                                predictOrbitalComponents = ExtrapolateOrbitParameters();
                            }
                            playPos++;
                            if (pathData.Count == playPos)
                            {
                                playback = false;
                            }
                        }
                    }
                    else
                    {
                        if (lastFreezeFrame != freezeFrame)
                        {
                            lastFreezeFrame = freezeFrame;
                            posError = CompareFreezePos();
                            predictVectorSpace = ExtrapolateOrbitVectorSpace();
                            predictOrbitalComponents = ExtrapolateOrbitParameters();
                        }
                        ourVessel.GoOnRails();
                        Planetarium.SetUniversalTime(pathData[freezeFrame].t);
                        ApplyOrbit(pathData[freezeFrame]);
                    }
                }

            }
        }


        private void SaveData()
        {
            string saveFile = Path.Combine(KSPUtil.ApplicationRootPath, "pathData.txt");
            if (File.Exists(saveFile))
            {
                File.Delete(saveFile);
            }
            using (FileStream fs = new FileStream(saveFile, FileMode.CreateNew))
            {
                using (BinaryWriter bw = new BinaryWriter(fs))
                {
                    bw.Write(pathData.Count);
                    foreach (OrbitalData od in pathData)
                    {
                        byte[] entrySize = od.ToBytes();
                        bw.Write(entrySize.Length);
                        bw.Write(entrySize);
                    }
                }
            }
            ScreenMessages.PostScreenMessage("Saved " + pathData.Count + " entries.", 5f, ScreenMessageStyle.UPPER_CENTER);
        }

        private void LoadData()
        {
            string saveFile = Path.Combine(KSPUtil.ApplicationRootPath, "pathData.txt");
            if (!File.Exists(saveFile))
            {
                return;
            }
            pathData = new List<OrbitalData>();
            using (FileStream fs = new FileStream(saveFile, FileMode.Open))
            {
                using (BinaryReader br = new BinaryReader(fs))
                {
                    int entries = br.ReadInt32();
                    for (int i = 0; i < entries; i++)
                    {
                        int entryLength = br.ReadInt32();
                        byte[] entryData = br.ReadBytes(entryLength);
                        OrbitalData entry = new OrbitalData(entryData);
                        pathData.Add(entry);
                    }
                }
            }
            ScreenMessages.PostScreenMessage("Loaded " + pathData.Count + " entries.", 5f, ScreenMessageStyle.UPPER_CENTER);
        }

        private void ApplyOrbit(OrbitalData orbitData)
        {
            Vessel updateVessel = FlightGlobals.ActiveVessel;
            updateVessel.GoOnRails();
            CelestialBody updateBody = FlightGlobals.GetBodyByName(orbitData.celestialBody);

            Orbit updateOrbit = orbitData.GetOrbit();
            updateOrbit.Init();
            updateOrbit.UpdateFromUT(Planetarium.GetUniversalTime());

            //Positional Error Tracking
            double latitude = updateBody.GetLatitude(updateOrbit.pos);
            double longitude = updateBody.GetLongitude(updateOrbit.pos);
            double altitude = updateBody.GetAltitude(updateOrbit.pos);
            updateVessel.latitude = latitude;
            updateVessel.longitude = longitude;
            updateVessel.altitude = altitude;
            updateVessel.protoVessel.latitude = latitude;
            updateVessel.protoVessel.longitude = longitude;
            updateVessel.protoVessel.altitude = altitude;
            VesselUtil.CopyOrbit(updateOrbit, updateVessel.orbitDriver.orbit);
            updateVessel.orbitDriver.updateFromParameters();

            /*
            if (!updateVessel.packed)
            {
                var fudgeVel = updateBody.inverseRotation ? updateBody.getRFrmVelOrbit(updateVessel.orbitDriver.orbit) : Vector3d.zero;
                updateVessel.SetWorldVelocity(updateVessel.orbitDriver.orbit.vel.xzy - fudgeVel - Krakensbane.GetFrameVelocity());
            }
            */

            updateVessel.SetRotation(updateVessel.mainBody.bodyTransform.rotation * orbitData.rot);
            if (updateVessel.packed)
            {
                updateVessel.srfRelRotation = orbitData.rot;
                updateVessel.protoVessel.rotation = updateVessel.srfRelRotation;
            }
        }

        private double CompareFreezePos()
        {
            double updateTime = pathData[playPos].t;
            Orbit vesselOrbit = pathData[(playPos / 10) * 10].GetOrbit();
            Orbit actualOrbit = pathData[playPos].GetOrbit();
            Vector3d vesselPos = vesselOrbit.getPositionAtUT(updateTime);
            Vector3d actualPos = actualOrbit.getPositionAtUT(updateTime);
            return Vector3d.Distance(vesselPos, actualPos);
        }

        private double CompareOrbit()
        {
            double updateTime = pathData[playPos].t;
            Orbit vesselOrbit = new Orbit(FlightGlobals.ActiveVessel.orbit);
            Orbit actualOrbit = pathData[playPos].GetOrbit();
            Vector3d vesselPos = vesselOrbit.getPositionAtUT(updateTime);
            Vector3d actualPos = actualOrbit.getPositionAtUT(updateTime);
            return Vector3d.Distance(vesselPos, actualPos);
        }

        private double ExtrapolateOrbitVectorSpace()
        {
            if (playPos < 10)
            {
                return 0;
            }
            double updateTime = pathData[playPos].t;
            Orbit pastOrbit = pathData[((playPos / 10) * 10) - 10].GetOrbit();
            Orbit currentOrbit = pathData[(playPos / 10) * 10].GetOrbit();
            Orbit predictOrbit = pathData[playPos].GetOrbit();
            Vector3d pastVector = pastOrbit.getPositionAtUT(updateTime);
            Vector3d currentVector = currentOrbit.getPositionAtUT(updateTime);
            Vector3d predictVector = predictOrbit.getPositionAtUT(updateTime);
            Vector3d guessVector = currentVector + (currentVector - pastVector);
            return Vector3d.Distance(predictVector, guessVector);
        }

        private double ExtrapolateOrbitParameters()
        {
            if (playPos < 10)
            {
                return 0;
            }
            double updateTime = pathData[playPos].t;
            Orbit pastOrbit = pathData[((playPos / 10) * 10) - 10].GetOrbit();
            Orbit currentOrbit = pathData[(playPos / 10) * 10].GetOrbit();
            Orbit predictOrbit = pathData[playPos].GetOrbit();

            double predictinclination = DoubleDiff(pastOrbit.inclination, currentOrbit.inclination);
            double predicteccentricity = DoubleDiff(pastOrbit.eccentricity, currentOrbit.eccentricity);
            double predictsemiMajorAxis = DoubleDiff(pastOrbit.semiMajorAxis, currentOrbit.semiMajorAxis);
            double predictLAN = DoubleDiff(pastOrbit.LAN, currentOrbit.LAN);
            double predictargumentOfPeriapsis = DoubleDiff(pastOrbit.argumentOfPeriapsis, currentOrbit.argumentOfPeriapsis);
            double predictmeanAnomalyAtEpoch = DoubleDiff(pastOrbit.meanAnomalyAtEpoch, currentOrbit.meanAnomalyAtEpoch);
            double predictt = DoubleDiff(pastOrbit.epoch, currentOrbit.epoch);
            Orbit guessOrbit = new Orbit(predictinclination, predicteccentricity, predictsemiMajorAxis, predictLAN, predictargumentOfPeriapsis, predictmeanAnomalyAtEpoch, predictt, currentOrbit.referenceBody);
            Vector3d predictVector = predictOrbit.getPositionAtUT(updateTime);
            Vector3d guessVector = guessOrbit.getPositionAtUT(updateTime);
            return Vector3d.Distance(predictVector, guessVector);
        }

        //A is old, B is new
        private double DoubleDiff(double a, double b)
        {
            return b + (b - a);
        }

        public void OnGUI()
        {
            windowRect = GUILayout.Window(12507593, windowRect, DrawWindow, "Posistioning");
            window2Rect = GUILayout.Window(12508593, window2Rect, DrawPredict, "Prediction");
        }

        public void DrawWindow(int windowID)
        {
            GUI.DragWindow(dragRect);
            if (playback)
            {
                if (GUILayout.Button("Cancel"))
                {
                    playback = false;
                    playPos = 0;
                }
                GUILayout.Label("Distance error: " + posError);
                GUILayout.Label("Vector predict error: " + predictVectorSpace);
                GUILayout.Label("Parameter predict error: " + predictOrbitalComponents);
                if (freezeFrame == -1)
                {
                    if (GUILayout.Button("Freeze"))
                    {
                        freezeFrame = playPos;
                    }
                }
                else
                {
                    if (GUILayout.Button("Unfreeze"))
                    {
                        freezeFrame = -1;
                    }
                    if (GUILayout.Button("-100"))
                    {
                        if (freezeFrame >= 100)
                        {
                            freezeFrame = freezeFrame - 100;
                            playPos = freezeFrame;
                        }
                    }
                    if (GUILayout.Button("-10"))
                    {
                        if (freezeFrame >= 10)
                        {
                            freezeFrame = freezeFrame - 10;
                            playPos = freezeFrame;
                        }
                    }
                    if (GUILayout.Button("-1"))
                    {
                        if (freezeFrame >= 1)
                        {
                            freezeFrame = freezeFrame - 1;
                            playPos = freezeFrame;
                        }
                    }
                    if (GUILayout.Button("+1"))
                    {
                        if (freezeFrame + 1 < pathData.Count)
                        {
                            freezeFrame = freezeFrame + 1;
                            playPos = freezeFrame;
                        }
                    }
                    if (GUILayout.Button("+10"))
                    {
                        if (freezeFrame + 10 < pathData.Count)
                        {
                            freezeFrame = freezeFrame + 10;
                            playPos = freezeFrame;
                        }
                    }
                    if (GUILayout.Button("+100"))
                    {
                        if (freezeFrame + 100 < pathData.Count)
                        {
                            freezeFrame = freezeFrame + 100;
                            playPos = freezeFrame;
                        }
                    }
                    GUILayout.Label("Orbital data:");
                    OrbitalData pathEntry = pathData[freezeFrame];
                    GUILayout.Label("inc: " + pathEntry.inclination);
                    GUILayout.Label("e: " + pathEntry.eccentricity);
                    GUILayout.Label("semiMajorAxis: " + pathEntry.semiMajorAxis);
                    GUILayout.Label("LAN: " + pathEntry.LAN);
                    GUILayout.Label("argumentOfPeriapsis: " + pathEntry.argumentOfPeriapsis);
                    GUILayout.Label("mep: " + pathEntry.meanAnomalyAtEpoch);
                    GUILayout.Label("t: " + pathEntry.t);
                    GUILayout.Label("longitude: " + pathEntry.lonPos);
                    GUILayout.Label("latitude: " + pathEntry.latPos);
                    GUILayout.Label("altitude: " + pathEntry.altPos);
                    GUILayout.Label("velX: " + pathEntry.vel.x);
                    GUILayout.Label("velY: " + pathEntry.vel.y);
                    GUILayout.Label("velZ: " + pathEntry.vel.z);
                }
            }
            else
            {
                if (recording)
                {
                    if (GUILayout.Button("Save"))
                    {
                        recording = false;
                        SaveData();
                    }
                    if (GUILayout.Button("Cancel"))
                    {
                        recording = false;
                        pathData = new List<OrbitalData>();
                    }
                }
                else
                {
                    if (GUILayout.Button("Playback"))
                    {
                        LoadData();
                        playback = true;
                        playbackStart = true;
                        playPos = 0;
                    }
                    /*
                    if (GUILayout.Button("Interpolate Stats"))
                    {
                        LoadData();
                        showInterpolate = !showInterpolate;
                        if (showInterpolate)
                        {
                            PredictOrbit();
                        }
                    }
                    */
                    if (GUILayout.Button("Record"))
                    {
                        recording = true;
                        pathData = new List<OrbitalData>();
                    }
                }
            }
        }

        public void DrawPredict(int windowID)
        {
            GUI.DragWindow(dragRect);
        }
    }
}
