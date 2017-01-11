#version 420 compatibility

struct V2T
{
    vec3 position;
    vec3 normal;
    vec3 texCoord;
};
#ifdef VertexShader //--------------------------------------

layout(location=1) in vec4 pos;
//(layout=1) in vec4 normal;
//(layout=2) in vec4 tex;
out V2T vdata;
void main()
{
    vdata.position = pos.xyz;
    //vdata.normal = normalize(normal.xyz);
	//vdata.texCoord = tex.xyz;
	//gl_TexCoord[0] = vec4(gl_Vertex.xyz*100.0f,1.0f)+100; //gl_TextureMatrix[0] * 
}
#endif

#ifdef FragmentShader //------------------------------------
out vec4 FragColor;
in V2T vdata;
//uniform vec3 LightPosition;
//uniform vec3 DiffuseMaterial;
//uniform vec3 AmbientMaterial;
const vec3 LIGHTPOS = vec3( 5., 10., 10. );

//layout(binding=0) uniform sampler3D tex;

float amplify(float d, float scale, float offset)
{
    d = scale * d + offset;
    d = 1 - exp2(-2*d*d);
    d = clamp(d, 0, 1);
    return d;
}

void main()
{
    vec3 N = normalize(vdata.normal);
    vec3 L = normalize(LIGHTPOS - vdata.position);
    float df = max(0,dot(N, L));
    //vec3 color = gl_FrontLightProduct[0].diffuse.rgb * (0.2+0.8*df);
    vec3 color = vec3(1,1,1);

  //color = texture(tex, vec3(vdata.position.st,vdata.position.z)).rgb;
    //color = texture(tex, vec3(0,0,0)).rgb;
  //color = vec3(vdata.texCoord.str);
  //color = gdexCoord.str;
  //color = vec3(1.0,0.5,0);
  FragColor = vec4(color, 1);
}
#endif