
Shader "Custom/ExploredAreaShader"
{
    Properties
    {
        _MainTex ("Base (RGB)", 2D) = "red" {}
        _AreaColor ("Area Color", Color) = (1,1,1,1)
        _Center ("Center", Vector) = (0,0,0,0)
        _Radius ("Radius", Range(0, 500)) = 75
    }
    SubShader
    {
        Tags { "Queue" = "Transparent" } // Set render queue to Transparent

        GrabPass { "_PrevColorTex" } // Grab the previous frame's color

        ZWrite Off // Disable writing to the depth buffer
        ZTest LEqual // Only write to pixels that pass the depth test (less or equal)

        CGPROGRAM
        #pragma surface surf Lambert alpha

        sampler2D _MainTex;
        sampler2D _PrevColorTex;
        fixed3 _AreaColor;
        float3 _Center;
        float _Radius;

        struct Input {
            float2 uv_MainText;
            float3 worldPos;

        };

        void surf (Input IN, inout SurfaceOutput o)
        {
            half4 c = tex2D (_MainTex, IN.uv_MainText);
            float dist = distance(_Center, IN.worldPos);

            half4 prevColor = tex2D(_PrevColorTex, IN.uv_MainText);


            if (dist < _Radius || (prevColor.r < 0.1 && prevColor.g < 0.1 && prevColor.b < 0.1))
            {
                o.Albedo = _AreaColor;
                c.a = 1;
            }
            else

            {
                o.Albedo = c.rgb;

            }

            o.Alpha = c.a;
        }
        ENDCG


    }
    FallBack "Diffuse"

}


