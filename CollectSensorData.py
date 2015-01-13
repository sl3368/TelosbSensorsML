
def write_reading_to_file(node,temp,light,humidity):
    with open('sensordata.txt',"a") as datafile:
        line=str(node)+" "+str(temp)+" "+str(light)+" "+str(humidity)+"\n"
        datafile.write(line)

    datafile.close()