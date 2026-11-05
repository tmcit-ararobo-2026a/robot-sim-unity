using System;
using UnityEngine;

namespace RobotSim.Environment
{
    public static class BucketPrefab
    {
        public const string MovingName = "Bucket_Moving";
        public const string LegacyMovingName = "Opponent_Moving_Bucket";
        public const string FixedPrefix = "bucket_fixed_";
        public const string AssetPath = "Assets/Prefab/PO24A_Bucket.prefab";

        public static Transform FindMoving(Transform opponent) =>
            opponent.Find(MovingName) ?? opponent.Find(LegacyMovingName);

#if UNITY_EDITOR
        public static GameObject LoadEditorAsset() => UnityEditor.AssetDatabase.LoadAssetAtPath<GameObject>(AssetPath);
#endif

        public static GameObject Instantiate(Transform parent, GameObject prefab)
        {
            if (prefab == null)
                throw new InvalidOperationException("Missing competition bucket prefab: " + AssetPath);
#if UNITY_EDITOR
            if (!Application.isPlaying)
                return (GameObject)UnityEditor.PrefabUtility.InstantiatePrefab(prefab, parent);
#endif
            return UnityEngine.Object.Instantiate(prefab, parent, false);
        }
    }
}
