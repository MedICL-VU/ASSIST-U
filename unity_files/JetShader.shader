Shader "Custom/JetShader"
{
    SubShader {
        Tags
        {
            "RenderType"="Opaque"
        }
        
        Pass
        {
            CGPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            #include "UnityCG.cginc"
            
            sampler2D _CameraDepthTexture;
      
            
            struct v2f
            {
                float4 pos : SV_POSITION;
                float4 scrPos:TEXCOORD1;
            };
            
            v2f vert (appdata_base v)
            {
                v2f o;
                o.pos = UnityObjectToClipPos (v.vertex);
                o.scrPos=ComputeScreenPos(o.pos);
                return o;
            }
            
            half4 RedToBlueColor(float value)
            {
                half4 color;
                color.r = 1.0 - value; // Red decreases as depth increases
                color.g = 0.0;        // No green component
                color.b = value;      // Blue increases as depth increases
                color.a = 1.0;
                return color;
            }
            half4 JetColor(float value)
	    {
	        half4 color;
	        // Red channel: starts increasing at halfway point, peaks at the end
	        color.r = smoothstep(0.5, 1.0, value);
	        // Green channel: increases, peaks in the middle, then decreases
	        color.g = smoothstep(0.0, 0.5, value) - smoothstep(0.75, 1.0, value);
	        // Blue channel: high at the beginning, decreases to zero by the midpoint
	        color.b = 1.0 - smoothstep(0.0, 0.5, value);
	        color.a = 1.0;
	        return color;
	    }
	    half4 Monochrome(float value)
	    {
	    	half4 depth;
                
                depth.r = value;
                depth.g = value;
                depth.b = value;
                depth.a = 1;
                return depth;
	    }


            half4 frag (v2f i) : COLOR
            {
                float depthValue = Linear01Depth(tex2Dproj(_CameraDepthTexture, UNITY_PROJ_COORD(i.scrPos)).r);

		 return Monochrome(depthValue);
                //return JetColor(depthValue);
            }

            
            ENDCG
        }
    }
    FallBack "Diffuse"
}

