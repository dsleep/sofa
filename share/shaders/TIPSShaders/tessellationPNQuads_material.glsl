#version 420 compatibility

struct V2T
{
    vec3 position;
    vec3 normal;
};

struct TC2E
{
    vec3 position;
    vec3 normal;
};

struct T2F
{
    vec3 position;
    vec3 normal;
    vec3 patchDistance;
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
out V2T vdata;

void main()
{
  vdata.position = gl_Vertex.xyz;
  vdata.normal = normalize(gl_Normal.xyz);
}
#endif

#ifdef TessellationControlShader //-----------------------
layout (vertices = 4) out;
uniform float TessellationLevel;
in V2T vdata[];
out TC2E tcdata[];

void main() {
	#define I gl_InvocationID
	tcdata[I].position = vdata[I].position;
	tcdata[I].normal = vdata[I].normal;

	gl_TessLevelOuter[0] = TessellationLevel;
	gl_TessLevelOuter[1] = TessellationLevel;
	gl_TessLevelOuter[2] = TessellationLevel;
	gl_TessLevelOuter[3] = TessellationLevel;
	gl_TessLevelInner[0] = TessellationLevel;
	gl_TessLevelInner[1] = TessellationLevel;
	#undef I
}
#endif

#ifdef TessellationEvaluationShader //-----------------------
layout(quads, equal_spacing, cw) in;
in TC2E tcdata[];
out T2F tedata;

void main() {
	vec3 p0 = tcdata[0].position;
	vec3 p1 = tcdata[1].position;
	vec3 p2 = tcdata[2].position;
	vec3 p3 = tcdata[3].position;

	vec3 n0 = tcdata[0].normal;
	vec3 n1 = tcdata[1].normal;
	vec3 n2 = tcdata[2].normal;
	vec3 n3 = tcdata[3].normal;

	float u = gl_TessCoord.x;
	float v = gl_TessCoord.y;

	vec3 b0 = p0;
	vec3 b1 = p1;
	vec3 b2 = p2;
	vec3 b3 = p3;

	float w01 = dot(p1 - p0, n0);
	float w10 = dot(p0 - p1, n1);
	float w12 = dot(p2 - p1, n1);
	float w21 = dot(p1 - p2, n2);
	float w23 = dot(p3 - p2, n2);
	float w32 = dot(p2 - p3, n3);
	float w30 = dot(p0 - p3, n3);
	float w03 = dot(p3 - p0, n0);

	vec3 b01 = (2.*p0 + p1 - w01*n0) / 3.;
	vec3 b10 = (2.*p1 + p0 - w10*n1) / 3.;
	vec3 b12 = (2.*p1 + p2 - w12*n1) / 3.;
	vec3 b21 = (2.*p2 + p1 - w21*n2) / 3.;
	vec3 b23 = (2.*p2 + p3 - w23*n2) / 3.;
	vec3 b32 = (2.*p3 + p2 - w32*n3) / 3.;
	vec3 b30 = (2.*p3 + p0 - w30*n3) / 3.;
	vec3 b03 = (2.*p0 + p3 - w03*n0) / 3.;

	vec3 q = b01 + b10 + b12 + b21 + b23 + b32 + b30 + b03;

	vec3 e0 = (2.*(b01 + b03 + q) - (b21 + b23)) / 18.;
	vec3 v0 = (4.*p0 + 2.*(p3 + p1) + p2) / 9.;
	vec3 b02 = e0 + (e0 - v0) / 2.;

	vec3 e1 = (2.*(b12 + b10 + q) - (b32 + b30)) / 18.;
	vec3 v1 = (4.*p1 + 2.*(p0 + p2) + p3) / 9.;
	vec3 b13 = e1 + (e1 - v1) / 2.;

	vec3 e2 = (2.*(b23 + b21 + q) - (b03 + b01)) / 18.;
	vec3 v2 = (4.*p2 + 2.*(p1 + p3) + p0) / 9.;
	vec3 b20 = e2 + (e2 - v2) / 2.;

	vec3 e3 = (2.*(b30 + b32 + q) - (b10 + b12)) / 18.;
	vec3 v3 = (4.*p3 + 2.*(p2 + p0) + p1) / 9.;
	vec3 b31 = e3 + (e3 - v3) / 2.;

	float bu0 = (1.-u) * (1.-u) * (1.-u);
	float bu1 = 3. * u * (1.-u) * (1.-u);
	float bu2 = 3. * u * u * (1.-u);
	float bu3 = u * u * u;

	float bv0 = (1.-v) * (1.-v) * (1.-v);
	float bv1 = 3. * v * (1.-v) * (1.-v);
	float bv2 = 3. * v * v * (1.-v);
	float bv3 = v * v * v;

	vec3 pos = bu0*(bv0*b0 + bv1*b01 + bv2*b10 + bv3*b1)
           + bu1*(bv0*b03 + bv1*b02 + bv2*b13 + bv3*b12)
           + bu2*(bv0*b30 + bv1*b31 + bv2*b20 + bv3*b21)
           + bu3*(bv0*b3 + bv1*b32 + bv2*b23 + bv3*b2);

	tedata.position = pos;

	float v01 = (2.*(dot(p1 - p0, n0 + n1) / dot(p1 - p0, p1 - p0)));
	float v12 = (2.*(dot(p2 - p1, n1 + n2) / dot(p2 - p1, p2 - p1)));
	float v23 = (2.*(dot(p3 - p2, n2 + n3) / dot(p3 - p2, p3 - p2)));
	float v30 = (2.*(dot(p0 - p3, n3 + n0) / dot(p0 - p3, p0 - p3)));

	vec3 n01 = normalize(n0 + n1 - v01*(p1 - p0));
	vec3 n12 = normalize(n1 + n2 - v12*(p2 - p1));
	vec3 n23 = normalize(n2 + n3 - v23*(p3 - p2));
	vec3 n30 = normalize(n3 + n0 - v30*(p0 - p3));

	vec3 n0123 = ((2.*(n01 + n12 + n23 + n30)) + (n0 + n1 + n2 + n3)) / 12.;

	float nu0 = (1.-u) * (1.-u);
	float nu1 = 2. * u * (1.-u);
	float nu2 = u * u;

	float nv0 = (1.-v) * (1.-v);
	float nv1 = 2. * v * (1.-v);
	float nv2 = v * v;

	tedata.normal = nu0*(nv0*n0 + nv1*n01 + nv2*n1)
                + nu1*(nv0*n30 + nv1*n0123 + nv2*n12)
                + nu2*(nv0*n3 + nv1*n23 + nv2*n2);

	gl_Position = gl_ModelViewProjectionMatrix * vec4(pos, 1);
}
#endif

#ifdef FragmentShader //------------------------------------
in T2F tedata;
uniform gl_MaterialParameters gl_FrontMaterial;
uniform gl_MaterialParameters gl_BackMaterial;

float basicNoise(vec2 co){
    return 0.5 + 0.5 * fract(sin(dot(co.xy ,vec2(12.9898,78.233))) * 43758.5453);
}

//For wetness effect
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

void main() {
	//Adapted from Ruiliang's Texture3D shader
	vec4 pos = gl_ModelViewMatrix * vec4(tedata.position, 1);
	vec3 mylightDir = normalize(vec3(0.1, 0.1, 0) - pos.xyz);//light position in ViewCoord

  //Generating a bump map from noise
  float noiseVal = fnoise(tedata.position.xyz) * 0.5 + 0.5;
  float E = 0.001;

  vec3 px = tedata.position.xyz;
  px.x += E;
  vec3 py = tedata.position.xyz;
  py.y += E;
  vec3 pz = tedata.position.xyz;
  pz.z += E;

  vec3 bump = vec3(fnoise(px)*0.5+0.5, fnoise(py)*0.5+0.5, fnoise(pz)*0.5+0.5);
  vec3 grad = vec3((bump.x-noiseVal)/E, (bump.y-noiseVal)/E, (bump.z-noiseVal)/E);

  vec3 pN = normalize(tedata.normal.xyz - grad);

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
                      + 1 * gl_FrontMaterial.specular * pow(max(0.0, clamp(dot(CamDir, ReflectedRay), -0.2, 1.0)), gl_FrontMaterial.shininess) * fresnel * basicNoise(tedata.position.xy) * 5;
}
#endif
