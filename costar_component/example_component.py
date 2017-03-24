from costar_component import MyComponent

class MyComponent(CostarComponent):
  def __init__(self, myargs, *args, **kwargs):
    super(MyComponent, self).__init__(*args, **kwargs)