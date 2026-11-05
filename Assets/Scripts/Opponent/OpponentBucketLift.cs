using RobotSim.Environment;
using UnityEngine;

namespace RobotSim.Opponent
{
    // Capture the previous physics pose in the sensors before positioning the next step.
    [DefaultExecutionOrder(100)]
    [DisallowMultipleComponent]
    public sealed class OpponentBucketLift : MonoBehaviour
    {
        // Competition stroke for this chassis. Even at the low stop the fixed
        // robot/mascot fit below the bucket bottom, hence below the 2 L mark.
        internal const float LowRimHeight = 1.64f;
        internal const float HighRimHeight = 2.03f;
        private const float CompetitionRobotHeight = 1.8f;
        private const float Speed = 0.08f;
        private const float TowerFootY = 0.25f;
        // The modeled 2 L mark is 50 mm above the bottom. Allow only 20 mm
        // of mounting hardware above the bottom to leave clearance below it.
        private const float MountAllowance = 0.02f;

        private OpponentRobotController _controller;
        private Transform _bucket, _tower, _plate;
        private Vector3 _bucketRest, _towerRest, _towerScale, _plateScale;
        private float _floorY, _rimOffset, _towerSpan;
        private bool _ascending = true;

        private void Awake() => _controller = GetComponent<OpponentRobotController>();

        private void FixedUpdate()
        {
            if (_controller == null || !_controller.BucketLiftEnabled) return;
            if (_bucket == null && !Initialize())
            {
                Debug.LogWarning("[OpponentBucketLift] Bucket, support, floor or height clearance is missing.", this);
                enabled = false;
                return;
            }

            float rim = _bucket.position.y + _rimOffset - _floorY;
            bool cycle = _controller.CycleBucketLift;
            float target = cycle
                ? (_ascending ? HighRimHeight : LowRimHeight)
                : Mathf.Clamp(_controller.TargetBucketRimHeight, LowRimHeight, HighRimHeight);
            float next = Mathf.MoveTowards(rim, target, Speed * Time.fixedDeltaTime);
            if (cycle && rim == target) _ascending = !_ascending;
            if (next == rim) return;

            SetOffset(_bucket.localPosition.y - _bucketRest.y + (next - rim) / transform.lossyScale.y);
            if (cycle && next == target) _ascending = !_ascending;
        }

        private bool Initialize()
        {
            _bucket = BucketPrefab.FindMoving(transform);
            _tower = transform.Find("Bucket_Support_Tower");
            _plate = _tower != null ? _tower.Find("Bucket_Base_MountPlate") : null;
            var renderer = _bucket != null ? _bucket.GetComponent<Renderer>() : null;
            FieldGroundPhysics floor = null;
            foreach (var candidate in FindObjectsByType<FieldGroundPhysics>(FindObjectsSortMode.None))
                if (candidate.gameObject.scene == gameObject.scene && candidate.FloorCollider != null)
                {
                    floor = candidate;
                    break;
                }
            if (renderer == null || _plate == null || floor == null ||
                Vector3.Dot(transform.up, Vector3.up) < 0.99999f ||
                Vector3.Dot(_bucket.up, Vector3.up) < 0.99999f ||
                transform.lossyScale.y <= 0) return false;

            _floorY = floor.FloorCollider.bounds.max.y;
            _rimOffset = renderer.bounds.max.y - _bucket.position.y;
            float rim = renderer.bounds.max.y - _floorY;
            if (rim < LowRimHeight || rim > HighRimHeight) return false;

            float lowBottom = _floorY + LowRimHeight - renderer.bounds.size.y;
            float towerLimit = Mathf.Min(renderer.bounds.min.y + MountAllowance,
                _floorY + CompetitionRobotHeight - (HighRimHeight - rim));
            foreach (var part in GetComponentsInChildren<Renderer>())
                if (part.enabled && !part.transform.IsChildOf(_bucket) &&
                    part.bounds.max.y > (part.transform.IsChildOf(_tower) ? towerLimit : lowBottom)) return false;
            foreach (var part in GetComponentsInChildren<Collider>())
                if (part.enabled && !part.isTrigger && !part.transform.IsChildOf(_bucket) &&
                    part.bounds.max.y > (part.transform.IsChildOf(_tower) ? towerLimit : lowBottom)) return false;

            _bucketRest = _bucket.localPosition;
            _towerRest = _tower.localPosition;
            _towerScale = _tower.localScale;
            _plateScale = _plate.localScale;
            _towerSpan = _towerRest.y + _plate.localPosition.y * _towerScale.y - TowerFootY;
            return _towerSpan > 0;
        }

        private void SetOffset(float offset)
        {
            _bucket.localPosition = _bucketRest + Vector3.up * offset;
            // Simulate a telescoping support about its deck attachment. The
            // mounting plate moves with the bucket and keeps its thickness.
            float extension = 1 + offset / _towerSpan;
            _tower.localScale = new Vector3(_towerScale.x, _towerScale.y * extension, _towerScale.z);
            _tower.localPosition = _towerRest + Vector3.up * ((_towerRest.y - TowerFootY) * (extension - 1));
            _plate.localScale = new Vector3(_plateScale.x, _plateScale.y / extension, _plateScale.z);
        }
    }
}
