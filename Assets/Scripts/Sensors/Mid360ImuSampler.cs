using System;
using UnityEngine;

namespace RobotSim.Sensors
{
    /// <summary>Immutable physics endpoints and a device-rate sampling clock.
    /// No measurement is ever reconstructed from a later publication-time pose.</summary>
    public sealed class Mid360ImuSampler
    {
        public struct State
        {
            public double Time;
            public Quaternion Rotation;
            public Vector3 VelocityWorld, AngularVelocityWorld;
        }
        public struct Sample
        {
            public double Time;
            public Quaternion Rotation;
            public Vector3 AngularVelocityWorld, SpecificForceWorld;
        }
        private State _previous;
        private bool _initialized;
        private double _epoch, _rate;
        private long _index;

        public void Capture(State current, double rate, Vector3 leverArmBaseUnity, Vector3 gravityWorld, Action<Sample> emit)
        {
            if (double.IsNaN(rate) || double.IsInfinity(rate) || rate <= 0 || rate > LivoxMid360Sim.MaxImuPublishRateHz)
                throw new ArgumentOutOfRangeException(nameof(rate));
            if (!_initialized)
            {
                _previous = current; _epoch = current.Time; _rate = rate; _index = 1; _initialized = true;
                return;
            }
            if (current.Time < _previous.Time) throw new InvalidOperationException("IMU physics time moved backwards");
            if (current.Time == _previous.Time) return; // Pause: no synthetic samples.
            if (rate != _rate) { _rate = rate; _epoch = _previous.Time; _index = 1; }
            double interval = current.Time - _previous.Time;
            Vector3 acceleration = (current.VelocityWorld - _previous.VelocityWorld) / (float)interval;
            Vector3 alpha = (current.AngularVelocityWorld - _previous.AngularVelocityWorld) / (float)interval;
            while (true)
            {
                double time = _epoch + _index / _rate;
                if (time > current.Time + 1e-10) break;
                float fraction = (float)((time - _previous.Time) / interval);
                Quaternion rotation = Quaternion.Slerp(_previous.Rotation, current.Rotation, fraction);
                Vector3 omega = Vector3.Lerp(_previous.AngularVelocityWorld, current.AngularVelocityWorld, fraction);
                Vector3 arm = rotation * leverArmBaseUnity;
                emit(new Sample
                {
                    Time = time, Rotation = rotation, AngularVelocityWorld = omega,
                    SpecificForceWorld = acceleration - gravityWorld + Vector3.Cross(alpha, arm) +
                        Vector3.Cross(omega, Vector3.Cross(omega, arm))
                });
                _index++;
            }
            _previous = current;
        }
    }
}
