# Program that generates a KML for a HAB mission as well as a KML (+png)
# Once generat the KMLs (and the png) just drag and drop in google earth to see hotzone
# for a "hotzone", understanding a hotzone as the most possible zone of landing
# use in moderations and just as mission planning! 
# Initially done by https://github.com/astromarc for Servet V mission

import requests
import datetime
import numpy as np
import dateutil.parser as parser
import pandas as pd
from pandas import json_normalize
import heatmap
import time


## Declaration of inputs
local_time_pred = '31 Oct 2021 06:00:00 +0000' # In UTC time
lauchlat = 41.65606
launchlong = -0.87734
descrate = 5 # in m/s 
numSim = 20 # number of simulations (100 for initial prediction is OK, but it takes a while)
variance = 1 #corresponds to the limit max and minim that ascend rate and descend rate will take (e.g., if ascend rate is 5m/s and variance is 1, simulations will consider from 4 to 6)
ascrate = 5 # in m/s 
burstaltitude = 30000 # in m


descrateNormal = np.random.normal(descrate, variance, numSim)
asrateNormal = np.random.normal(ascrate, variance, numSim)
local_time_pred  = parser.parse(local_time_pred)
local_time_pred = (local_time_pred.astimezone().isoformat())
# api-endpoint
URL = "http://predict.cusf.co.uk/api/v1/"
from datetime import datetime, timezone
current_time = datetime.now(timezone.utc).astimezone().isoformat() # if you want to use current UTC time

if launchlong <0:	launchlong = launchlong +360


# defining a params dict for the parameters to be sent to the API
PARAMS = {'launch_latitude':lauchlat, 'launch_longitude':launchlong, 'launch_datetime':local_time_pred, #Use local_time_pred or current_time
          'ascent_rate' : ascrate, 'descent_rate' : descrate, 'burst_altitude' : burstaltitude}
# sending get request and saving the response as response object
response = requests.get(url = URL, params = PARAMS)
print(response)

# extracting data in json format
data = response.json()
prediction = data['prediction']
ascend = prediction[0]
descend = prediction[1]



df_ascend = json_normalize(ascend['trajectory'])
df_descend = json_normalize(descend['trajectory'])
df = pd.concat([df_ascend, df_descend])
df = df.astype('string')

f = open('splineBlue.kml', 'w')
f.write('<?xml version="1.0" encoding="UTF-8"?>\n')
f.write('<kml xmlns="http://www.opengis.net/kml/2.2">\n')
f.write("	<Document>\n")
f.write("	<name>Initial Prediction</name>\n")
f.write('	<Style id="style1">\n')
f.write("		<LineStyle>\n")
f.write("			<color>641478FF</color>\n")
f.write("			<width>4</width>\n")
f.write("		</LineStyle>\n")
f.write("		<PolyStyle>\n")
f.write("			<color>281478FF</color>\n")
f.write("		</PolyStyle>\n")
f.write("	</Style>\n")
f.write("		<Placemark>\n")
f.write("			<name> Initial Prediction </name>\n")
f.write("			<styleUrl>#style1</styleUrl> \n")
f.write("   		<LineString>\n")
f.write("   		<extrude>1</extrude>\n")
f.write("   		<tessellate>1</tessellate>\n")
f.write("   		<altitudeMode>absolute</altitudeMode>\n")
f.write("			<coordinates>")
for index, row in df.iterrows():
  	f.write("			"+row['longitude']+","+row['latitude']+","+row['altitude']+"\n")
f.write("			</coordinates>\n")
f.write("			</LineString>\n")
f.write("		</Placemark>\n")
f.write("</Document>")
f.write("</kml>")
f.close()

# we calculate the Hotzone (potential landing sites), first the descend and then append the rsults in the ascend
df_hotZoneDesc = pd.DataFrame(columns = ['altitude', 'datetime','latitude','longitude'])
df_hotZoneAsc = pd.DataFrame(columns = ['altitude', 'datetime','latitude','longitude'])
df_hotZoneTotal = pd.DataFrame(columns = ['altitude', 'datetime','latitude','longitude'])

i = 0;
for x in descrateNormal:
	for y in asrateNormal:
		PARAMS = {'launch_latitude':lauchlat, 'launch_longitude':launchlong, 'launch_datetime':local_time_pred, 'ascent_rate' : y, 'descent_rate' : x, 'burst_altitude' : burstaltitude}
		response = requests.get(url = URL, params = PARAMS)
		# extracting data in json format
		data = response.json()
		prediction = data['prediction']
		descend = prediction[1]
		df_descend = json_normalize(descend['trajectory'])
		df_descend = df_descend.astype('string')
		df_tail = df_descend.tail(1)
		df_hotZoneDesc = pd.concat([df_hotZoneDesc,df_tail])
		print((numSim*numSim)-i)
		i = i+1;
 
# for x in asrateNormal:
# 	PARAMS = {'launch_latitude':lauchlat, 'launch_longitude':launchlong, 'launch_datetime':local_time_pred, 'ascent_rate' : x, 'descent_rate' : descrate, 'burst_altitude' : burstaltitude}
# 	response = requests.get(url = URL, params = PARAMS)
# 	# extracting data in json format
# 	data = response.json()
# 	prediction = data['prediction']
# 	descend = prediction[1]
# 	df_descend = json_normalize(descend['trajectory'])
# 	df_descend = df_descend.astype('string')
# 	df_tail = df_descend.tail(1)
# 	df_hotZoneAsc = pd.concat([df_hotZoneAsc,df_tail])

lat_hotZoneDesc = df_hotZoneDesc[['latitude']].to_numpy()
lon_hotZoneDesc = df_hotZoneDesc[['longitude']].to_numpy()

#lat_hotZoneAsc = df_hotZoneAsc[['latitude']].to_numpy()
#lon_hotZoneAsc = df_hotZoneAsc[['longitude']].to_numpy()

ptsDesc = [(float(lon_hotZoneDesc[x]), float(lat_hotZoneDesc[x])) for x in range(len(lon_hotZoneDesc))]
#ptsAsc =  [(float(lon_hotZoneAsc[x]), float(lat_hotZoneAsc[x])) for x in range(len(lon_hotZoneAsc))]

pts = ptsDesc#+ptsAsc

hm = heatmap.Heatmap()
hm.heatmap(pts)
hm.saveKML("initialHotzone.kml")