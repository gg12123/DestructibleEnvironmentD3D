cbuffer PerObjectConstantBuffer : register(b0)
{
	matrix modelToWorldMatrix;
	float4 colour;
};

cbuffer PerSceneConstantBuffer : register(b1)
{
	matrix vPMatrix;
	float4 lightPosition;
	float4 viewDirection;
};

struct VertexShaderInput
{
	float4 pos : POSITION;
	float4 normal : NORMAL0;
};

struct PixelShaderInput
{
	float4 pos : SV_POSITION;
	float3 color : COLOR0;
};

PixelShaderInput main(VertexShaderInput input)
{
	PixelShaderInput output;

	float4 pos = float4(input.pos.x, input.pos.y, input.pos.z, 1.0f);
	pos = mul(modelToWorldMatrix, pos);

	float3 worldPos = float3(pos.x, pos.y, pos.z);

	pos = mul(vPMatrix, pos);
	output.pos = pos;

	float4 normal = mul(modelToWorldMatrix, input.normal);

	float3 n = float3(normal.x, normal.y, normal.z);
	float3 l = float3(lightPosition.x, lightPosition.y, lightPosition.z);
	float diffuse = dot(n, normalize(l - worldPos));

	output.color = diffuse * float3(1.0f, 0.0f, 0.0f);// float3(colour.x, colour.y, colour.z);

	return output;
}