local root = sofa.simulation:newGraph('root')
root.gravity = { 0, 0, -10}
root:newObject{ 'VisualStyle',  displayFlags="showBehaviorModels showCollisionModels" }
--[[ LCP stuff
root:newObject{ 'GenericConstraintSolver' , tolerance = '1e-3', maxIterations = '1000' }
root:newObject{ 'FreeMotionAnimationLoop'  }
--]]
root:newObject{ 'CollisionPipeline' , depth = '6' }
root:newObject{ 'BruteForceDetection'  }
root:newObject{ 'MinProximityIntersection' , alarmDistance = '0.5', contactDistance = '0.3' }
root:newObject{ 'CollisionResponse', response='default' }
root:newObject{ 'RequiredPlugin' , name = 'SurfLabHexahedralIGA' }
root:newObject{ 'EulerImplicit' }
root:newObject{ 'CGLinearSolver', iterations='25', tolerance='1.e-9', threshold = '1.e-9' }

function uniformGrid(lower,upper,step)
    p = {}
    for z = lower[3],upper[3],step do
      for y = lower[2],upper[2],step do
        for x = lower[1],upper[1],step do
          table.insert(p, x)
          table.insert(p, y)
          table.insert(p, z)
        end
      end
    end
    return p
end

function makeTricubic(parent,layers,length,width,translation)
    local cube = parent:newChild('cube')
    local topo = cube:newObject{ 'TricubicBezierSetTopologyContainer' }

    l = length
    w = width
    ro = l*3+1
    sh = (l*3+1)*(w*3+1)


    bz = {}
    for t=0,3*layers-1,3 do
      for u=0,3*w-1,3 do
        for v = 0,3*l-1,3 do
          for tt = 0,3 do
            for uu = 0,3 do
              for vv = 0,3 do
                table.insert(bz, (tt+t)*sh + (uu+u)*ro + (vv+v))
              end
            end
          end
        end
      end
    end

    c = {}
    for i=0,sh-1 do
        table.insert(c,i+3*layers*sh)
    end

    topo.position = uniformGrid({-1.5*l,-1.5*w,-3*layers},{1.5*l,1.5*w,0},1)
    topo.beziers = bz 
    local mo = cube:newObject{ 'MechanicalObject', name = 'MO' }
    mo.translation = translation
    cube:newObject{ 'UniformMass', mass = '1' }
    fc = cube:newObject{ 'FixedConstraint' }
    fc.indices = c
    cube:newObject{ 'Point' }
    cube:newObject{ 'Line' }
    cube:newObject{ 'TricubicBezierForceField', youngModulus = '1000', poissonRatio = '0.45', rayleighStiffness='0' }
    cube:newObject{ 'OglModel', name='Visual'}
    cube:newObject{ 'IdentityMapping', input='@MO',output='@Visual' }
    return cube
end

function makeTrilinear(parent,layers,width,length,translation)
    local cube = parent:newChild('cube')
    local topo = cube:newObject{ 'HexahedronSetTopologyContainer' }
    
    l = length
    w = width
    ro = l*3+1
    sh = (l*3+1)*(w*3+1)
    
    h = {}
    for t=0,3*layers-1 do
      for u = 0,w*3-1 do
        for v = 0,l*3-1 do
          table.insert(h, t * sh + u * ro+ v)
          table.insert(h, t * sh + u * ro+ v+1)
          table.insert(h, t * sh + (u+1) * ro+ v+1)
          table.insert(h, t * sh + (u+1) * ro+ v)
          table.insert(h, (t+1) * sh + u * ro+ v)
          table.insert(h, (t+1) * sh + u * ro+ v+1)
          table.insert(h, (t+1) * sh + (u+1) * ro+ v+1)
          table.insert(h, (t+1) * sh + (u+1) * ro+ v)
        end
      end
    end 
    
    c = {}
    for i=0,sh-1 do
        table.insert(c,i+3*layers*sh)
    end
     
    
    topo.position = uniformGrid({-1.5*l,-1.5*w,-3*layers},{1.5*l,1.5*w,0},1)
    topo.hexahedra = h
    local mo = cube:newObject{ 'MechanicalObject', name = 'MO' }
    mo.translation = translation
    cube:newObject{ 'UniformMass', mass = '1' }
    fc = cube:newObject{ 'FixedConstraint' }
    fc.indices = c
    cube:newObject{ 'Point' }
    cube:newObject{ 'Line' }
    cube:newObject{ 'TrilinearFEMForceField', youngModulus = '1000', poissonRatio = '0.45', rayleighStiffness='0' }
    cube:newObject{ 'OglModel', name='Visual'}
    cube:newObject{ 'IdentityMapping', input='@MO',output='@Visual' }
    return cube
end

    
makeTricubic(root,3,2,2,{-5, 0, 0})
makeTrilinear(root,3,2,2,{5,0,0})
return root
