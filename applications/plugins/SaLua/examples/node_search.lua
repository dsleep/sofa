
function printNodes(obs)
  for i, x in ipairs(obs) do
    print(i .. ": " .. x.name .. '   ' .. x:className())
  end
end

r = controller:getContext()

print("looking for pri")
printNodes( r:getObjects({ "pri" }) )

print("looking for some")
printNodes( r:findChildNodes({ "some" }) )

print("looking for some thing")
printNodes( r:findChildNodes({ "some", "thing" }) )

