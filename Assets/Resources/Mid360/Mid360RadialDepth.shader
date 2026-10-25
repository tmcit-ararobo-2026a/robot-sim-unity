Shader "Hidden/Mid360/RadialDepth"
{
    Properties
    {
        [PerRendererData] _Mid360Reflectivity ("Calibrated 905 nm reflectivity", Float) = 0
    }
    SubShader
    {
        Pass
        {
            ZWrite On
            ZTest LEqual
            Cull Off
            CGPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            #include "UnityCG.cginc"
            float _Mid360Reflectivity;
            struct v2f
            {
                float4 position : SV_POSITION;
                float3 worldPosition : TEXCOORD0;
            };
            v2f vert(float4 vertex : POSITION)
            {
                v2f o;
                o.position = UnityObjectToClipPos(vertex);
                o.worldPosition = mul(unity_ObjectToWorld, vertex).xyz;
                return o;
            }
            float2 frag(v2f i) : SV_Target
            {
                return float2(distance(i.worldPosition, _WorldSpaceCameraPos), _Mid360Reflectivity);
            }
            ENDCG
        }
    }
    FallBack Off
}
