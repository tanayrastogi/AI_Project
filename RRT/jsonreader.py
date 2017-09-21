import json

class Data:

	def __init__(self,filename):
		with open(filename) as json_data:
			settings = json.load(json_data)

		self.phi_max = settings.get("phi_max")
		self.a_max = settings.get("a_max")
		self.omega_max = settings.get("omega_max")
		self.v_max = settings.get("v_max")
		self.L_car = settings.get("L_car")
		self.start_pos = settings.get("start_pos")
		self.start_vel = settings.get("start_vel")
		self.goal_pos = settings.get("goal_pos")
		self.goal_vel = settings.get("goal_vel")
		self.k_friction = settings.get("k_friction")
		self.boundary = settings.get('boundary_polygon')

		self.obsticles = list()
		for key in settings:
			#print(key)
			if('polygon' in key and key != 'boundary_polygon'):
				#print("Obsticle: " + key)
				self.obsticles.append(settings.get(key))


def main():
	data = Data('problem_2.json')

	print(data.obsticles)

if __name__ == '__main__':
    main()
