#version 420 compatibility

struct V2F {
  vec3 position;
  vec3 normal;
  vec2 texcoord;
};

#ifdef VertexShader //--------------------------------------
out V2F vdata;

void main()
{
  gl_Position = ftransform();

  vdata.position = gl_Vertex.xyz;
  vdata.normal = normalize(gl_Normal.xyz);
  vdata.texcoord.xy = gl_MultiTexCoord0.xy;
}
#endif

#ifdef FragmentShader //------------------------------------
in V2F vdata;
uniform sampler2D colorTexture;

float basicNoise(vec2 co){
    return 0.5 + 0.5 * fract(sin(dot(co.xy ,vec2(12.9898,78.233))) * 43758.5453);
}

//3D Value Noise generator by Morgan McGuire @morgan3d
//https://www.shadertoy.com/view/4dS3Wd
float hash(float n) { return fract(sin(n) * 1e4); }
float hash(vec2 p) { return fract(1e4 * sin(17.0 * p.x + p.y * 0.1) * (0.1 + abs(sin(p.y * 13.0 + p.x)))); }

float noise(vec3 x) {
	const vec3 step = vec3(110, 241, 171);

	vec3 i = floor(x);
	vec3 f = fract(x);

	// For performance, compute the base input to a 1D hash from the integer part of the argument and the
	// incremental change to the 1D based on the 3D -> 1D wrapping
    float n = dot(i, step);

	vec3 u = f * f * (3.0 - 2.0 * f);
	return mix(mix(mix( hash(n + dot(step, vec3(0, 0, 0))), hash(n + dot(step, vec3(1, 0, 0))), u.x),
		   mix( hash(n + dot(step, vec3(0, 1, 0))), hash(n + dot(step, vec3(1, 1, 0))), u.x), u.y),
	       mix(mix( hash(n + dot(step, vec3(0, 0, 1))), hash(n + dot(step, vec3(1, 0, 1))), u.x),
		   mix( hash(n + dot(step, vec3(0, 1, 1))), hash(n + dot(step, vec3(1, 1, 1))), u.x), u.y), u.z);
}

//Fractional Brownian Motion
#define NUM_OCTAVES 2

float fnoise(vec3 x) {
	float v = 0.0;
	float a = 0.5;
	vec3 shift = vec3(100);
	for (int i = 0; i < NUM_OCTAVES; ++i) {
		v += a * noise(x);
		x = x * 2.0 + shift;
		a *= 0.5;
	}
	return v;
}

void main()
{
  vec2 in_texcoord = vdata.texcoord;
  vec4 color = texture2D(colorTexture, in_texcoord);
  //Adapted from Ruiliang's Texture3D shader
	vec4 pos = gl_ModelViewMatrix * vec4(vdata.position, 1);
	vec3 mylightDir = normalize(vec3(0.1, 0.1, 0) - pos.xyz);//light position in ViewCoord

  //Generating a bump map from noise
  float noiseVal = fnoise(vec3(in_texcoord,1)) * 0.5 + 0.5;
  float E = 0.001;

  vec3 px = vec3(in_texcoord,1);
  px.x += E;
  vec3 py = vec3(in_texcoord,1);
  py.y += E;
  vec3 pz = vec3(in_texcoord,1);
  pz.z += E;

  vec3 bump = vec3(fnoise(px)*0.5+0.5, fnoise(py)*0.5+0.5, fnoise(pz)*0.5+0.5);
  vec3 grad = vec3((bump.x-noiseVal)/E, (bump.y-noiseVal)/E, (bump.z-noiseVal)/E);

  vec3 pN = normalize(vdata.normal.xyz - grad);

  vec4 Nr = gl_ModelViewMatrixInverseTranspose * vec4(pN, 1);
  vec3 myN = normalize(Nr.xyz);//This step is so important that the Rasteration step will use interpolattion to get N, which must be normalized.
  vec3 ReflectedRay = reflect(mylightDir, myN );
  vec3 CamDir = normalize(pos.xyz);//Cam position in ViewCoord is 0,0,0

  //Fresnel Term
  float F0 = 0.5;
  vec3 h = normalize(CamDir + mylightDir);
  float base = 1 - dot(CamDir, h);
  float exponential = pow(base, 5.0);
  float fresnel = exponential + F0 * (1.0 - exponential);

  // color below = ambient + specular
  gl_FragColor.xyz = vec3(0.1,0.05,0.0) + 0.25 * color.xyz
                      + 0.4 * color.xyz * clamp(dot(mylightDir, myN), -0.2, 1.0)
                      + 1.0 * vec3(0.8, 0.8, 0.8) * pow(max(0.0, clamp(dot(CamDir, ReflectedRay), -0.2, 1.0)), 100) * fresnel * 5;
}
#endif
