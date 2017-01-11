#version 120

uniform sampler2D colorTexture;
uniform sampler3D colorTexture3;
//uniform vec4 lightPosition;
varying vec4 lightVec;
varying vec3 lightDir;
uniform float mapDimX;
uniform int orientation; // 0 -> Horizontal, 1 -> Vertical
varying vec3 N;
varying vec3 viewVector;
varying vec3 lightpos;
varying vec4 pos;

void main() {
    //this will be our RGBA sum
    vec4 sum = vec4(0.0);

    //our original texcoord for this fragment
    vec2 tc = gl_TexCoord[0].xy;

    //the amount to blur, i.e. how far off center to sample from 
    //1.0 -> blur by one pixel
    //2.0 -> blur by two pixels, etc.
    float blur = 1.0/mapDimX; 
    vec2 dir;
     if(orientation == 0)
         dir = vec2(0, 1.0);
    else
         dir = vec2(1.0, 0);

    //the direction of our blur
    //(1.0, 0.0) -> x-axis blur
    //(0.0, 1.0) -> y-axis blur
    float hstep = dir.x;
    float vstep = dir.y;

    //apply blurring, using a 9-tap filter with predefined gaussian weights

    sum += texture2D(colorTexture, vec2(tc.x - 4.0*blur*hstep, tc.y - 4.0*blur*vstep)) * 0.0162162162;
    sum += texture2D(colorTexture, vec2(tc.x - 3.0*blur*hstep, tc.y - 3.0*blur*vstep)) * 0.0540540541;
    sum += texture2D(colorTexture, vec2(tc.x - 2.0*blur*hstep, tc.y - 2.0*blur*vstep)) * 0.1216216216;
    sum += texture2D(colorTexture, vec2(tc.x - 1.0*blur*hstep, tc.y - 1.0*blur*vstep)) * 0.1945945946;

    sum += texture2D(colorTexture, vec2(tc.x, tc.y)) * 0.2270270270;

    sum += texture2D(colorTexture, vec2(tc.x + 1.0*blur*hstep, tc.y + 1.0*blur*vstep)) * 0.1945945946;
    sum += texture2D(colorTexture, vec2(tc.x + 2.0*blur*hstep, tc.y + 2.0*blur*vstep)) * 0.1216216216;
    sum += texture2D(colorTexture, vec2(tc.x + 3.0*blur*hstep, tc.y + 3.0*blur*vstep)) * 0.0540540541;
    sum += texture2D(colorTexture, vec2(tc.x + 4.0*blur*hstep, tc.y + 4.0*blur*vstep)) * 0.0162162162;

vec4 tex2Dgauss3(sampler2D tex, vec2 texcoord, vec2 dim)  { return tex2DgaussN(tex, texcoord, dim, 2 , 0.7); }
vec4 tex2Dgauss5(sampler2D tex, vec2 texcoord, vec2 dim)  { return tex2DgaussN(tex, texcoord, dim, 3 , 0.9); }
vec4 tex2Dgauss7(sampler2D tex, vec2 texcoord, vec2 dim)  { return tex2DgaussN(tex, texcoord, dim, 4 , 1.1); }
vec4 tex2Dgauss9(sampler2D tex, vec2 texcoord, vec2 dim)  { return tex2DgaussN(tex, texcoord, dim, 5 , 1.4); }
vec4 tex2Dgauss11(sampler2D tex, vec2 texcoord, vec2 dim) { return tex2DgaussN(tex, texcoord, dim, 6 , 1.8); }
vec4 tex2Dgauss13(sampler2D tex, vec2 texcoord, vec2 dim) { return tex2DgaussN(tex, texcoord, dim, 7 , 2.2); }
vec4 tex2Dgauss15(sampler2D tex, vec2 texcoord, vec2 dim) { return tex2DgaussN(tex, texcoord, dim, 8 , 2.6); }
vec4 tex2Dgauss19(sampler2D tex, vec2 texcoord, vec2 dim) { return tex2DgaussN(tex, texcoord, dim, 10, 3.5); }
vec4 tex2Dgauss23(sampler2D tex, vec2 texcoord, vec2 dim) { return tex2DgaussN(tex, texcoord, dim, 12, 4.5); }
vec4 tex2Dgauss27(sampler2D tex, vec2 texcoord, vec2 dim) { return tex2DgaussN(tex, texcoord, dim, 14, 5.5); }
vec4 tex2Dgauss31(sampler2D tex, vec2 texcoord, vec2 dim) { return tex2DgaussN(tex, texcoord, dim, 16, 6.6); }

void main()
{
	//vec2 texcoord = gl_TexCoord[0].xy;
  vec3 texcoord3 = gl_TexCoord[0].xyz;
	//vec4 color = texture2D(colorTexture, texcoord);
  
  // vec3 ReflectedRay = reflect( NlightDir, N );
   vec4 color = texture3D(colorTexture3, texcoord3);
	// //vec2 dim;
  // float dp = clamp( dot( N, NlightDir ), 0., 1. );
	// if( dp < 0. ) dp = 0.;
	// //else if( dp<.5 ) dp = .5;
	// //else if( dp<.9 ) dp = .9;
	// else if ( dp>.9 )dp = 1.5;
	// if(orientation == 0)
		// dim = vec2(0, 1.0/mapDimX);
	// else 
		// dim = vec2(1.0/mapDimX, 0);
    
	//color = tex2Dgauss7(colorTexture, texcoord, dim);
  //color = vec4(N,0.0);
	//gl_FragColor = vec4(1,0,0,0.0);
  //float dpv = N[2];
  vec3 mylightDir = normalize(vec3(0.1, 0.1, 0) - pos.xyz);//light position in ViewCoord
  vec3 myN = normalize(N);
  //N = normalize(N);
  vec3 ReflectedRay = reflect(mylightDir, myN );//
  vec3 CamDir = normalize(pos.xyz);//Cam position in ViewCoord is 0,0,0
  gl_FragColor.xyz = vec3(0.2,0.1,0.1) + 0.5 * color.xyz + 1 * color.xyz * dot(CamDir,ReflectedRay);//dot( N,viewVector );//dot(viewVector,N);//color * 0.5;
	//gl_FragColor.xyz = N;
  // if( dpv>.3 )
		// gl_FragColor.xyz += 0.1 + vec3(0.5,0.5,0.0) * dp;//.9*gl_FrontLightProduct[0].diffuse.xyz * dp;
	// else 
    // gl_FragColor.xyz = vec3( 0., 0., 0. );
}

