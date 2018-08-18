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

	pos = mul(pos, modelToWorldMatrix);
	pos = mul(pos, vPMatrix);
	output.pos = pos;

	output.color = float3(1.0f, 0.0f, 0.0f);

	return output;
}