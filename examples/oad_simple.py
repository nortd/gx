
import gx

print gx.settings.VERSION

c = gx.random_curve(6)
c.scale(400)
c.translate(700,0,1000)

r = gx.get_openabbrobot()
r.add_curve(c)
r.close()