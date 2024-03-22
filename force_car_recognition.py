filenames = ['/data/openpilot/selfdrive/car/top_tmp/HondaCars', '/data/openpilot/selfdrive/car/top_tmp/HyundaiCars', '/data/openpilot/selfdrive/car/top_tmp/SubaruCars',
             '/data/openpilot/selfdrive/car/top_tmp/ToyotaCars', '/data/openpilot/selfdrive/car/top_tmp/VolkswagenCars']

with open('/data/openpilot/selfdrive/car/top_tmp/Cars', 'w') as outfile:
  for names in filenames:
    with open(names) as infile:
      outfile.write(infile.read())
    outfile.write("\n")
