import yaml

default_map = {
    'default_map': 'teknokent.yaml',
    'default_keepout': 'teknokent_keepout.yaml'
}

with open('data.yaml', 'w') as file:
    yaml.dump(default_map, file)

with open('data.yaml', 'r') as newfile:
    loaded = yaml.safe_load(newfile)

print(type(loaded))
print("data has been written to 'data.yaml' ")