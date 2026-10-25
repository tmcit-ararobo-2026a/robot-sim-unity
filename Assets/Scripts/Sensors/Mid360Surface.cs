using UnityEngine;

namespace RobotSim.Sensors
{
    /// <summary>
    /// Calibrated 905 nm reflectivity in the device's 0..255 encoding. Visible RGB
    /// albedo is not a NIR measurement. Unassigned surfaces report unknown (zero).
    /// </summary>
    [ExecuteAlways]
    public sealed class Mid360Surface : MonoBehaviour
    {
        [Tooltip("Calibrated device reflectivity: 0..150 diffuse 0..100%; 151..255 retroreflective. Default 0 is uncalibrated.")]
        [Range(0, 255)] public int reflectivity;
        private static readonly int ReflectivityId = Shader.PropertyToID("_Mid360Reflectivity");
        private MaterialPropertyBlock _block;
        private int _applied = -1;

        private void OnEnable() => Apply();
        private void OnValidate() => _applied = -1;
        private void Update()
        {
            if (_applied != reflectivity) Apply();
        }

        public void Apply()
        {
            var renderer = GetComponent<Renderer>();
            if (renderer == null) return;
            if (_block == null) _block = new MaterialPropertyBlock();
            renderer.GetPropertyBlock(_block);
            renderer.SetPropertyBlock(SetReflectivity(_block, Mathf.Clamp(reflectivity, 0, 255)));
            _applied = reflectivity;
        }

        private static MaterialPropertyBlock SetReflectivity(MaterialPropertyBlock block, int value)
        {
            block.SetFloat(ReflectivityId, value);
            return block;
        }

        private void OnDisable()
        {
            var renderer = GetComponent<Renderer>();
            if (renderer == null || _block == null) return;
            renderer.GetPropertyBlock(_block);
            renderer.SetPropertyBlock(SetReflectivity(_block, 0));
            _applied = -1;
        }
    }
}
