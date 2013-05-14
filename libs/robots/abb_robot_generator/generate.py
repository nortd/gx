



template = open("abb_template.wrl").read()

# base = open("irb2600-base.wrl").read()
# link1 = open("irb2600-link1.wrl").read()
# link2 = open("irb2600-link2.wrl").read()
# link3 = open("irb2600-link3.wrl").read()
# link4 = open("irb2600-link4.wrl").read()
# link5 = open("irb2600-link5.wrl").read()
# link6 = open("irb2600-link6.wrl").read()

base = open("irb2600-base-high.wrl").read()
link1 = open("irb2600-link1-high.wrl").read()
link2 = open("irb2600-link2-high.wrl").read()
link3 = open("irb2600-link3-high.wrl").read()
link4 = open("irb2600-link4-high.wrl").read()
link5 = open("irb2600-link5-high.wrl").read()
link6 = open("irb2600-link6-high.wrl").read()

template = template.replace('{{ base }}', base)
template = template.replace('{{ link1 }}', link1)
template = template.replace('{{ link2 }}', link2)
template = template.replace('{{ link3 }}', link3)
template = template.replace('{{ link4 }}', link4)
template = template.replace('{{ link5 }}', link5)
template = template.replace('{{ link6 }}', link6)

# template = template.replace('diffuseColor     1 1 1', 'diffuseColor     0.2 0.2 0.2')  # dark gray
template = template.replace('diffuseColor     1 1 1', 'diffuseColor     1 0.41 0')  # oragne
template = template.replace("	skyColor", '# 	skyColor')

out = open("irb2600.wrl", 'w')
out.write(template)