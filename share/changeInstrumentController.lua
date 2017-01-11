
-- Find all haptic devices

r = controller:getContext()
haptics = {} -- global variable
-- h is one node with tag "haptic" from getContext()
for i, h in pairs(r:findChildNodes({ 'haptic' })) do
  hp = { activated = false, node = h, currentInstrument = 1 }
  hp.instrumentNodes = h:findChildNodes({'instrument'})
  haptics[i] = hp
  print('Haptic node: ' , h.name )
  for i = 1, #hp.instrumentNodes do
    hp.instrumentNodes[i]:setActive(i == 1)
    print('Instrument: ', hp.instrumentNodes[i].name)
  end
end

function changeInstrument(h)
  h.instrumentNodes[h.currentInstrument]:setActive(false)
  h.currentInstrument = math.fmod(h.currentInstrument, #h.instrumentNodes) + 1  
  print('Changing instrument TO: ' .. h.node.name .. ' ' .. h.instrumentNodes[h.currentInstrument].name)
  h.instrumentNodes[h.currentInstrument]:setActive(true)
  h.instrumentNodes[h.currentInstrument]:init()
 
end


-- onKeyPressed is only called if you hold Control down
-- so for '[' one must type Ctrl+[
function handlers.onKeyPressed(c)
  if c == 'C' then
    if haptics[1] then 
      changeInstrument(haptics[1])
    end
  elseif c == 'H' then
    if haptics[2] then
      changeInstrument(haptics[2])
    end
  end
end

Tool2cliks = 0
Tool1cliks = 0
function handlers.onHaptic(c,d)-- means button state, c means
   -- print('c:',c)
   -- print('d:',d)
  if c == 0 and d == 2 then
    if Tool2cliks >= 4 then
      changeInstrument(haptics[1])
      Tool2cliks = -1
    else
      Tool2cliks = Tool2cliks + 1 
    end
  end
end
  -- if c == 1 and d == 2 then
    -- if Tool1cliks <= 2 then
      -- Tool1cliks = Tool1cliks + 1
    -- else 
      -- Tool1cliks = -1
      -- changeInstrument(haptics[2])
    -- end
  -- end

