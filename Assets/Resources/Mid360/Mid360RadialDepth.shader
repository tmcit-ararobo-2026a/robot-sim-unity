Shader "Hidden/Mid360/RadialDepth"
{
    Properties
    {
        [PerRendererData] _Mid360Reflectivity ("Calibrated 905 nm reflectivity", Float) = 0
        [PerRendererData] _Mid360Transmission ("905 nm transmission", Float) = 0
        [PerRendererData] _Mid360Backscatter ("905 nm receiver coupling", Float) = 0
    }
    SubShader
    {
        Pass
        {
            ZWrite On
            ZTest LEqual
            Cull Off
            CGPROGRAM
            #pragma target 4.0
            #pragma vertex vert
            #pragma fragment frag
            #include "UnityCG.cginc"
            #include "Mid360Optics.hlsl"
            float _Mid360Reflectivity;
            float _Mid360Transmission;
            float _Mid360Backscatter;
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
                float3 ray = i.worldPosition - _WorldSpaceCameraPos;
                float range = length(ray);
                if (_Mid360Transmission > 0)
                {
                    float3 normal = normalize(cross(ddx(i.worldPosition), ddy(i.worldPosition)));
                    float3 direction = normalize(ray);
                    float incidence = saturate(abs(dot(normal, direction)));
                    float probability = TransparentReturnProbability(_Mid360Transmission, _Mid360Backscatter,
                        _Mid360Reflectivity, incidence, range, TenPercentRangeMeters);
                    if (!AcceptReturn(probability, direction, 0)) discard;
                }
                return float2(range, _Mid360Reflectivity);
            }
            ENDCG
        }
    }
    FallBack Off
}
