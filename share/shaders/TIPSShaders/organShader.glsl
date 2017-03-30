#version 420 compatibility

struct V2F {
  vec3 position;
  vec3 normal;
};

struct gl_MaterialParameters
{
   vec4 emission;    // Ecm
   vec4 ambient;     // Acm
   vec4 diffuse;     // Dcm
   vec4 specular;    // Scm
   float shininess;  // Srm
};

#ifdef VertexShader //--------------------------------------
out V2F vdata;

void main()
{
  gl_Position = ftransform();

  vdata.position = gl_Vertex.xyz;
  vdata.normal = normalize(gl_Normal.xyz);
}
#endif

#ifdef FragmentShader //------------------------------------
in V2F vdata;

uniform gl_MaterialParameters gl_FrontMaterial;
uniform gl_MaterialParameters gl_BackMaterial;

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
#define NUM_OCTAVES 1

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
  //Adapted from Ruiliang's Texture3D shader
	vec4 pos = gl_ModelViewMatrix * vec4(vdata.position, 1);
	vec3 mylightDir = normalize(vec3(0.1, 0.1, 0) - pos.xyz);//light position in ViewCoord

  //Generating a bump map from noise
  float noiseVal = fnoise(vdata.position) * 0.5 + 0.5;
  float E = 0.001;

  vec3 px = vdata.position;
  px.x += E;
  vec3 py = vdata.position;
  py.y += E;
  vec3 pz = vdata.position;
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
  gl_FragColor = gl_FrontMaterial.ambient
                      + 0.5 * gl_FrontMaterial.diffuse * clamp(dot(mylightDir, myN), -0.2, 1.0)
                      + 1 * gl_FrontMaterial.specular * pow(max(0.0, clamp(dot(CamDir, ReflectedRay), -0.2, 1.0)), gl_FrontMaterial.shininess) * fresnel * basicNoise(vdata.position.xy) * 5;
}
#endif
