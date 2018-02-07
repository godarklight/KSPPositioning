using System;
using MessageStream2;
using UnityEngine;
namespace KSPPositioning
{
    public class OrbitalData
    {
        public OrbitalData(Vessel copyFrom)
        {
            inclination = copyFrom.orbit.inclination;
            eccentricity = copyFrom.orbit.eccentricity;
            semiMajorAxis = copyFrom.orbit.semiMajorAxis;
            LAN = copyFrom.orbit.LAN;
            argumentOfPeriapsis = copyFrom.orbit.argumentOfPeriapsis;
            meanAnomalyAtEpoch = copyFrom.orbit.meanAnomalyAtEpoch;
            t = copyFrom.orbit.epoch;
            celestialBody = copyFrom.orbit.referenceBody.name;
            vel = Quaternion.Inverse(copyFrom.mainBody.bodyTransform.rotation) * copyFrom.srf_velocity;
            rot = copyFrom.srfRelRotation;
        }
        public double inclination;
        public double eccentricity;
        public double semiMajorAxis;
        public double LAN;
        public double argumentOfPeriapsis;
        public double meanAnomalyAtEpoch;
        public double t;
        public double lonPos;
        public double latPos;
        public double altPos;
        public Vector3d vel;
        public Quaternion rot;
        public string celestialBody;

        public Orbit GetOrbit()
        {
            CelestialBody returnBody = FlightGlobals.GetBodyByName(celestialBody);
            if (returnBody == null)
            {
                return null;
            }
            return new Orbit(inclination, eccentricity, semiMajorAxis, LAN, argumentOfPeriapsis, meanAnomalyAtEpoch, t, returnBody);
        }

        public OrbitalData(byte[] orbitalData)
        {
            using (MessageReader mr = new MessageReader(orbitalData))
            {
                inclination = mr.Read<double>();
                eccentricity = mr.Read<double>();
                semiMajorAxis = mr.Read<double>();
                LAN = mr.Read<double>();
                argumentOfPeriapsis = mr.Read<double>();
                meanAnomalyAtEpoch = mr.Read<double>();
                t = mr.Read<double>();
                celestialBody = mr.Read<string>();
                lonPos = mr.Read<double>();
                latPos = mr.Read<double>();
                altPos = mr.Read<double>();
                vel = new Vector3d(mr.Read<double>(), mr.Read<double>(), mr.Read<double>());
                rot = new Quaternion(mr.Read<float>(), mr.Read<float>(), mr.Read<float>(), mr.Read<float>());
            }
        }

        public byte[] ToBytes()
        {
            byte[] retVal = null;
            using (MessageWriter mw = new MessageWriter())
            {
                mw.Write<double>(inclination);
                mw.Write<double>(eccentricity);
                mw.Write<double>(semiMajorAxis);
                mw.Write<double>(LAN);
                mw.Write<double>(argumentOfPeriapsis);
                mw.Write<double>(meanAnomalyAtEpoch);
                mw.Write<double>(t);
                mw.Write<string>(celestialBody);
                mw.Write<double>(lonPos);
                mw.Write<double>(latPos);
                mw.Write<double>(altPos);
                mw.Write<double>(vel.x);
                mw.Write<double>(vel.y);
                mw.Write<double>(vel.z);
                mw.Write<float>(rot.x);
                mw.Write<float>(rot.y);
                mw.Write<float>(rot.z);
                mw.Write<float>(rot.w);
                retVal = mw.GetMessageBytes();
            }
            return retVal;
        }
    }

}
