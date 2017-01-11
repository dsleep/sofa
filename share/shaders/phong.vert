//phong

varying vec3 normal;
varying vec3 lightDir;
varying vec4 SunCoord;
varying vec3 viewVector;
uniform vec3 lightpos;
uniform vec3 campos;

void main()
{
	SunCoord = gl_TextureMatrix[0] * gl_Vertex;
	vec4 pos = gl_ModelViewMatrix * gl_Vertex;
  vec3 lightpos = vec3(0.1, 0.1, 0);
	//vec3 lightpos = vec3(-6.2816, -3.287, 0.91);//gl_LightSource[0].position.xyz;
	lightpos = gl_ModelViewMatrix * vec4( lightpos, 1 );
	lightDir =   lightpos - pos.xyz ;
	//viewVector = normalize( gl_Vertex.xyz - gl_ModelViewMatrix[3].xyz );
	viewVector = -( campos - pos.xyz );
  //viewVector = -( campos - gl_Vertex.xyz );
	
	normal = gl_Normal;
	gl_Position = ftransform();
}
