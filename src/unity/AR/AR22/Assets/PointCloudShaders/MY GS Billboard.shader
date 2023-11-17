//This is a modified version of a shader published by smb02dunnal on the Unity forums:
//https://forum.unity3d.com/threads/billboard-geometry-shader.169415/

Shader "Custom/GS Billboard"
{
	Properties
	{
		_PointSize("PointSize", Range(0, 0.1)) = 0.01
	}

	SubShader
	{
	Pass
	{
		Tags{ "RenderType" = "Opaque" }
		LOD 200

		CGPROGRAM

		#pragma target 5.0
		#pragma vertex vert
		#pragma geometry geom
		#pragma fragment frag
		
		#include "UnityCG.cginc" 

		// **************************************************************
		// Data structures												*
		// **************************************************************
		struct VertInput
		{
			float4 position : POSITION;
			half4 color : COLOR;
			half4 normal : NORMAL;
		};

		struct VertToGeom {
			float4 position : SV_POSITION;
			half4 color : COLOR;
			float4 right : TEXCOORD0;
			float4 up : TEXCOORD1;
		};

		struct VertOutput
		{
			float4 position : SV_POSITION;
			half4 color : COLOR;
		};


		// **************************************************************
		// Vars															*
		// **************************************************************

		float _PointSize;

		// **************************************************************
		// Shader Programs												*
		// **************************************************************

		// Vertex Shader ------------------------------------------------
		
		VertToGeom vert(VertInput v) {
			VertToGeom o;
			o.position = v.position;
			o.color = v.color;

			float3 upDir = normalize(UNITY_MATRIX_IT_MV[1].xyz);
			float3 viewDir = normalize(UNITY_MATRIX_IT_MV[2].xyz);
			float3 fwDir = viewDir; 

			float3 rightDir = normalize(cross(fwDir, upDir));

			o.up = float4(upDir * _PointSize, 0);
			o.right = float4(rightDir * _PointSize, 0);
			return o;
		}

		[maxvertexcount(4)]
		void geom(point VertToGeom input[1], inout TriangleStream<VertOutput> outTriangles) {
			float4 splatCenter = input[0].position;
			half4 splatColor = input[0].color;
			float4 right = input[0].right;
			float4 up = input[0].up;
			
			float3 worldPos = mul(unity_ObjectToWorld, splatCenter).xyz;

			VertOutput vo1;
			vo1.position = UnityObjectToClipPos(splatCenter - right + up);
			vo1.color = splatColor;
			
			VertOutput vo2;
			vo2.position = UnityObjectToClipPos(splatCenter + right + up);
			vo2.color = splatColor;
			
			VertOutput vo3;
			vo3.position = UnityObjectToClipPos(splatCenter + right - up);
			vo3.color = splatColor;
			
			VertOutput vo4;
			vo4.position = UnityObjectToClipPos(splatCenter - right - up);
			vo4.color = splatColor;
			
			outTriangles.Append(vo1);
			outTriangles.Append(vo2);
			outTriangles.Append(vo4);
			outTriangles.Append(vo3);
		}

		half4 frag(VertOutput v) : COLOR 
		{
			return v.color;
		}

		ENDCG
	}
	}
}
