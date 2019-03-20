function handlers.onKeyPressed(c)
  print(c .. ' is pressed ')
end
handlers.onLoad =  function ()
  print('loading complete in lua controller')
end
print('this is has run')
controller.listening = 1