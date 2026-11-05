using UnityEngine;

namespace RobotSim.Sensors
{
    /// <summary>
    /// 905 nm reflectivity in the device's 0..255 encoding. Visible RGB
    /// albedo is not a NIR measurement. Unassigned surfaces report unknown (zero).
    /// </summary>
    [ExecuteAlways]
    public sealed class Mid360Surface : MonoBehaviour
    {
        [Tooltip("Device reflectivity: 0..150 diffuse 0..100%; 151..255 retroreflective. Default 0 is uncalibrated.")]
        [Range(0, 255)] public int reflectivity;
        [Tooltip("Zero selects opaque returns, one forces pass-through. Intermediate values set the transmission probability in the legacy angular model.")]
        [Range(0, 1)] public float transmission;
        [Tooltip("Estimated fraction of the nominal reflection coupled back to the receiver. Zero uses the legacy angular model.")]
        [Range(0, 1)] public float backscatter;

        internal static bool WithinDetectionRange(float distance, float reflectivity, float referenceRange)
        {
            // Approximate a fixed detection threshold with inverse-square return
            // strength. Code 15 is 10%; cap extrapolation for retroreflectors.
            return reflectivity == 0 || referenceRange <= 0 ||
                distance * distance <= referenceRange * referenceRange * Mathf.Min(reflectivity, 150) / 15f;
        }

        internal static float TransparentReturnProbability(float transmission, float backscatter, float reflectivity,
            float incidence, float distance, float referenceRange)
        {
            if (transmission >= 1) return 0;
            if (backscatter <= 0)
            {
                float incidenceFourth = incidence * incidence * incidence * incidence;
                return (1 - transmission) * incidenceFourth * incidenceFourth;
            }

            // Scale the coupled echo against the 10% diffuse reference (code 15).
            // A soft threshold approximates variability from roughness and receiver
            // noise. Limit near-field gain where the device's response is unspecified.
            float rangeRatio = (referenceRange > 0 ? referenceRange : LivoxMid360Sim.TenPercentRangeMeters) /
                Mathf.Max(distance, 1f);
            float signal = backscatter * Mathf.Min(reflectivity, 150) / 15f *
                incidence * incidence * rangeRatio * rangeRatio;
            return signal / (1 + signal);
        }

        internal static bool AcceptReturn(float probability, Vector3 direction, float phase)
        {
            // Stable across CPU/GPU, readback retries and coincident shell triangles.
            uint hash = unchecked((uint)System.BitConverter.SingleToInt32Bits(direction.x)
                ^ (uint)System.BitConverter.SingleToInt32Bits(direction.y) * 0x9e3779b9u
                ^ (uint)System.BitConverter.SingleToInt32Bits(direction.z) * 0x85ebca6bu
                ^ (uint)System.BitConverter.SingleToInt32Bits(phase));
            hash = unchecked((hash ^ (hash >> 16)) * 0x7feb352du);
            hash = unchecked((hash ^ (hash >> 15)) * 0x846ca68bu);
            float sample = ((hash ^ (hash >> 16)) & 0x00ffffffu) / 16777216f;
            return sample < probability;
        }

        private static readonly int ReflectivityId = Shader.PropertyToID("_Mid360Reflectivity");
        private static readonly int TransmissionId = Shader.PropertyToID("_Mid360Transmission");
        private static readonly int BackscatterId = Shader.PropertyToID("_Mid360Backscatter");
        private MaterialPropertyBlock _block;
        private int _applied = -1;
        private float _appliedTransmission;
        private float _appliedBackscatter;

        private void OnEnable() => Apply();
        private void OnValidate() => _applied = -1;
        private void Update()
        {
            if (_applied != reflectivity || _appliedTransmission != transmission || _appliedBackscatter != backscatter) Apply();
        }

        public void Apply()
        {
            var renderer = GetComponent<Renderer>();
            if (renderer == null) return;
            if (_block == null) _block = new MaterialPropertyBlock();
            renderer.GetPropertyBlock(_block);
            SetOptics(renderer, Mathf.Clamp(reflectivity, 0, 255), Mathf.Clamp01(transmission), Mathf.Clamp01(backscatter));
            _applied = reflectivity;
            _appliedTransmission = transmission;
            _appliedBackscatter = backscatter;
        }

        private void SetOptics(Renderer renderer, int reflectivityCode, float transmittedFraction, float receiverCoupling)
        {
            _block.SetFloat(ReflectivityId, reflectivityCode);
            _block.SetFloat(TransmissionId, transmittedFraction);
            _block.SetFloat(BackscatterId, receiverCoupling);
            renderer.SetPropertyBlock(_block);
        }

        private void OnDisable()
        {
            var renderer = GetComponent<Renderer>();
            if (renderer == null || _block == null) return;
            renderer.GetPropertyBlock(_block);
            SetOptics(renderer, 0, 0, 0);
            _applied = -1;
        }
    }
}
