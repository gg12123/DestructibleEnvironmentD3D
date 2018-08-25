cbuffer PerObjectConstantBuffer : register(b0)
{
	matrix modelToWorldMatrix;
};

cbuffer PerSceneConstantBuffer : register(b1)
{
	matrix vPMatrix;
	float3 lightPosition;
	float3 viewDirection;
};

struct VertexShaderInput
{
	float3 pos : POSITION;
	float3 normal : NORMAL0;
};

struct PixelShaderInput
{
	float4 pos : SV_POSITION;
	float3 color : COLOR0;
};

PixelShaderInput main(VertexShaderInput input)
{
	PixelShaderInput output;

	float4 pos = float4(input.pos, 1.0f);
	pos = mul(modelToWorldMatrix, pos);

	float3 worldPos = float3(pos.x, pos.y, pos.z);

	pos = mul(vPMatrix, pos);
	output.pos = pos;

	float4 normal = float4(input.normal, 0.0f);
	normal = mul(modelToWorldMatrix, normal);

	float3 n = float3(normal.x, normal.y, normal.z);
	float diffuse = dot(n, normalize(lightPosition - worldPos));

	output.color = diffuse * float3(1.0f, 0.0f, 0.0f);

	return output;
}