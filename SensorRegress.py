from sklearn.linear_model import LinearRegression

class Node:

    def __init__(self,id):
        self.sensor_id=id;
        self.measurement_list=[]

    def addMeasurement(self,temp,light,humidity):
        measurement=[]
        measurement.append(temp)
        measurement.append(light)
        measurement.append(humidity)
        self.measurement_list.append(measurement)

def addMeasurementToNode(nodes,node,temp,light,humidity):

    if node not in nodes:
        newNode=Node(node)
        newNode.addMeasurement(temp,light,humidity)
        nodes[newNode.sensor_id]=newNode
    else:
        nodes[node].addMeasurement(temp,light,humidity)

    return nodes


def findNodeCoefficients(node):
    node_coefficients=[]
    node_measurements=node.measurement_list
    lag_one=[]
    lag_two=[]
    lag_three=[]
    lag_four=[]
    lag_five=[]
    real=[]

    for i in range(15,len(node_measurements)):
        lag_one.append([node_measurements[i-1][0],node_measurements[i-6][0],node_measurements[i-11][0]])
        lag_two.append([node_measurements[i-2][0],node_measurements[i-7][0],node_measurements[i-12][0]])
        lag_three.append([node_measurements[i-3][0],node_measurements[i-8][0],node_measurements[i-13][0]])
        lag_four.append([node_measurements[i-4][0],node_measurements[i-9][0],node_measurements[i-14][0]])
        lag_five.appen([node_measurements[i-5][0],node_measurements[i-10][0],node_measurements[i-15][0]])
        real.append(node_measurements[i][0])

    lag_one_LM=LinearRegression()
    lag_one_LM.fit(lag_one,real)
    one_coeff=lag_one_LM.coef_.tolist()
    one_coeff.append(lag_one_LM.intercept_)

    lag_two_LM=LinearRegression()
    lag_two_LM.fit(lag_two,real)
    two_coeff=lag_two_LM.coef_.tolist()
    two_coeff.append(lag_two_LM.intercept_)

    lag_three_LM=LinearRegression()
    lag_three_LM.fit(lag_three,real)
    three_coeff=lag_three_LM.coef_.tolist()
    three_coeff.append(lag_three_LM.intercept_)

    lag_four_LM=LinearRegression()
    lag_four_LM.fit(lag_four,real)
    four_coeff=lag_four_LM.coef_.tolist()
    four_coeff.append(lag_four_LM.intercept_)

    lag_five_LM=LinearRegression()
    lag_five_LM.fit(lag_five,real)
    five_coeff=lag_five_LM.coef_.tolist()
    five_coeff.append(lag_five_LM.intercept_)

    node_coefficients.append(one_coeff)
    node_coefficients.append(two_coeff)
    node_coefficients.append(three_coeff)
    node_coefficients.append(four_coeff)
    node_coefficients.append(five_coeff)

    return node_coefficients