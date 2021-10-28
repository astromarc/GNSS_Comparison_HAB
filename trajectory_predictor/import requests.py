import requests
import datetime
# api-endpoint
URL = "http://predict.cusf.co.uk/api/v1/"
from datetime import datetime, timezone
local_time = datetime.now(timezone.utc).astimezone()


lauchlat = 41.65606
launchlong = -0.87734
descrate = 5
burstaltitude = 30000
ascrate = 5
current_time = local_time

if launchlong <0:	launchlong = launchlong +360


# defining a params dict for the parameters to be sent to the API
PARAMS = {'launch_latitude':lauchlat, 'launch_longitude':launchlong, 'launch_datetime':local_time.isoformat(), 'ascent_rate' : ascrate, 'descent_rate' : descrate, 'burst_altitude' : burstaltitude}
# sending get request and saving the response as response object
response = requests.get(url = URL, params = PARAMS)
print(response)

# extracting data in json format
data = response.json()
prediction = data['prediction']
ascend = prediction[0]
descend = prediction[1]


import pandas as pd
from pandas import json_normalize
df_ascend = json_normalize(ascend['trajectory'])
df_descend = json_normalize(descend['trajectory'])
print(df_ascend)
df_ascend.append(df_descend)
df = df_ascend 
df = df.astype('string')

f = open('splineBlue.kml', 'w')
f.write('<?xml version="1.0" encoding="UTF-8"?>\n')
f.write('<kml xmlns="http://www.opengis.net/kml/2.2">\n')
f.write("	<Document>\n")
f.write("	<name>First Prediction</name>\n")
f.write('	<Style id="style1">\n')
f.write("		<LineStyle>\n")
f.write("			<color>64B40014</color>\n")
f.write("			<width>4</width>\n")
f.write("		</LineStyle>\n")
f.write("		<PolyStyle>\n")
f.write("			<color>5aFFFFFF</color>\n")
f.write("		</PolyStyle>\n")
f.write("	</Style>\n")
f.write("		<Placemark>\n")
f.write("			<name>First Prediction </name>\n")
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