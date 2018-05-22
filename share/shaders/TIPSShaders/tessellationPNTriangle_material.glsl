#version 420 compatibility

struct V2T
{
    vec3 position;
    vec3 normal;
};

struct TC2E
{
    vec3 b3;
    vec3 b21x3, b12x3;
    vec3 n2;
    vec3 n11;
};

struct TPatch
{
    vec3 b111x6;
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
layout (vertices = 3) out;
uniform float TessellationLevel;
//const float TessellationLevel = 6.;
in V2T vdata[];
out TC2E tcdata[];
out patch TPatch tpdata;

void main()
{
//    const int I = gl_InvocationID;
#define I gl_InvocationID
    const int J = (I+1)%3;
    vec3 p1 = vdata[I].position;

    tcdata[I].b3 = p1;

    tcdata[I].n2 = vdata[I].normal;
    vec3 dp = vdata[J].position - vdata[I].position;

    vec3 n12 = vdata[I].normal + vdata[J].normal;
    tcdata[I].n11 = normalize( n12*dot(dp,dp) + dp*(2.*dot(dp,n12)) );

    float w12 = dot(dp,vdata[I].normal);
    float w21 = -dot(dp,vdata[J].normal);

    tcdata[I].b21x3 = (2.*vdata[I].position + vdata[J].position - w12*vdata[I].normal);
    tcdata[I].b12x3 = (2.*vdata[J].position + vdata[I].position - w21*vdata[J].normal);

    gl_TessLevelOuter[I] = TessellationLevel;

    barrier();
    //float ee = (tcdata[0].b21x3[I] + tcdata[0].b12x3[I] + tcdata[1].b21x3[I] + tcdata[1].b12x3[I] + tcdata[2].b21x3[I] + tcdata[2].b12x3[I]);
    //float vv = (tcdata[0].b3[I] + tcdata[1].b3[I] + tcdata[2].b3[I]);
    //p.b111x6[I] = (ee/2. + vv);
    if (I==0)
    {
        vec3 eex18 = (tcdata[0].b21x3 + tcdata[0].b12x3 + tcdata[1].b21x3 + tcdata[1].b12x3 + tcdata[2].b21x3 + tcdata[2].b12x3);
        vec3 vvx3 = (tcdata[0].b3 + tcdata[1].b3 + tcdata[2].b3);
        tpdata.b111x6 = (eex18/2. - vvx3);

        gl_TessLevelInner[0] = TessellationLevel;
    }
#undef I
}

#endif
#ifdef TessellationEvaluationShader //-----------------------
layout(triangles, equal_spacing, cw) in;
in TC2E tcdata[];
in patch TPatch tpdata;
out T2F tedata;

void main()
{
    tedata.patchDistance = gl_TessCoord;
    float u = gl_TessCoord.x, v = gl_TessCoord.y, w = gl_TessCoord.z;

    float u2 = u*u, v2 = v*v, w2 = w*w;
    tedata.normal = tcdata[0].n2*(u2) + tcdata[1].n2*(v2) + tcdata[2].n2*(w2) +
        tcdata[0].n11*(u*v) + tcdata[1].n11*(v*w) + tcdata[2].n11*(w*u);
    vec3 pos = tcdata[0].b3*(u2*u) + tcdata[1].b3*(v2*v) + tcdata[2].b3*(w2*w) +
        tcdata[0].b21x3*(u2*v) + tcdata[0].b12x3*(u*v2) +
        tcdata[1].b21x3*(v2*w) + tcdata[1].b12x3*(v*w2) +
        tcdata[2].b21x3*(w2*u) + tcdata[2].b12x3*(w*u2) +
        tpdata.b111x6*(u*v*w);

    tedata.position = pos;

    gl_Position = gl_ModelViewProjectionMatrix * vec4(pos, 1);
}

#endif

#ifdef FragmentShader //------------------------------------
in T2F tedata;

const vec3 LIGHTPOS = vec3( -50., 10., 150. );
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
  gl_FragColor = gl_FrontMaterial.ambient
                      + 0.5 * gl_FrontMaterial.diffuse * clamp(dot(mylightDir, myN), -0.2, 1.0)
                      + 1.0 * vec4(0.9, 0.9, 0.9, 1.0) * pow(max(0.0, clamp(dot(CamDir, ReflectedRay), -0.2, 1.0)), 30) * fresnel * 5;;
 }
#endif
