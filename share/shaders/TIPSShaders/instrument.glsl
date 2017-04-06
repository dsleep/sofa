#version 120

#ifdef VertexShader //--------------------------------------
varying vec3 normal;
varying vec3 lightDir;

void main()
{
	vec4 pos = gl_ModelViewMatrix * gl_Vertex;
	vec3 lightpos = gl_LightSource[0].position.xyz;
	lightDir =  lightpos - pos.xyz;
	normal = gl_NormalMatrix * gl_Normal;
	gl_Position = ftransform();
}
#endif

#ifdef FragmentShader //------------------------------------
varying vec3 normal;
varying vec3 lightDir;
vec3 boundaryColor = vec3( 0., 0., 0. );
void main()
{
	vec3 NNormal = normalize( normal );
	vec3 NlightDir = normalize( lightDir );
	vec3 ReflectedRay = reflect( NlightDir, NNormal );
	
	vec3 color;
	float dp = clamp( dot( NNormal, NlightDir ), 0., 1. );
	if( dp < 0. ) dp = 0.;
	else if( dp<.5 ) dp = .5;
	else if( dp<.9 ) dp = .9;
	else dp = 1.5;
	
	float dpv = NNormal[2];
	if( dpv>.4 )
		gl_FragColor.xyz = .1 + .9*gl_FrontLightProduct[0].diffuse.xyz * dp;
	else 
    gl_FragColor.xyz = boundaryColor;
}
#endif



















































































































































































































































































































































































































































































































































































































































