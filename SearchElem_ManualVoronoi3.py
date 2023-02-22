
def isclose(a, b):
    result = False
    if abs(abs(a)-abs(b)) <= 0.4:
        result= True
    else:
        result = False
    return result

def isclose2(a, b):
    result = False
    if abs(abs(a)-abs(b)) <= 0.4:
        result= True
    else:
        result = False

    print ("a:{} , b:{} , result :{} ".format(a,b,result))
    return result


def find_indices(list_to_check, item_to_find1, item_to_find2):
    indices1 = []
    indices2 = []
    #print(len(list_to_check))
    for idx, value in enumerate(list_to_check):
        if isclose(value,item_to_find1):    
            indices1.append(idx)
        if isclose(value,item_to_find2):
            indices2.append(idx)
    return indices1,indices2



def getSharedBoundary(cell1_x,cell1_y,cell2_x,cell2_y):

    sharedBoundaryStart = []
    sharedBoundaryEnd = []
    #rint("cell")
    #print(cell1_x)
    ##print(cell1_y)
    #print(cell2_x)
    #print(cell2_y)
    for i in range(len(cell1_x)-1):
        #print("i:{} ".format(i))

        xComparison,x1Comparison = find_indices(cell2_x,cell1_x[i],cell1_x[i+1])
        yComparison,y1Comparison = find_indices(cell2_y,cell1_y[i],cell1_y[i+1])
        #yComparison = []
        #y1Comparison = []
        #print(xComparison)
        #print(x1Comparison)
        #print(yComparison)
        #print(y1Comparison)
        if(xComparison and x1Comparison and yComparison and y1Comparison):
            res_startPoint = set(xComparison) & set(yComparison)
            #print("res_start:{}".format(res_startPoint))
            res_endPoint = set(x1Comparison) & set(y1Comparison)
            if(res_startPoint and res_endPoint):
                sharedBoundaryStart.append( round(cell1_x[i],2))
                sharedBoundaryStart.append(round(cell1_y[i],2))
                sharedBoundaryEnd.append(round(cell1_x[i+1],2))
                sharedBoundaryEnd.append(round(cell1_y[i+1],2))
    #print("resilt")
    #print(sharedBoundaryStart)
    #print(sharedBoundaryEnd)
    if not sharedBoundaryStart:
        return []
    else:
        return sharedBoundaryStart[0],sharedBoundaryStart[1],sharedBoundaryEnd[0],sharedBoundaryEnd[1]
    

'''
c1_x = [0.0, 4.999999999999999, 3.4949999999999988, 0.0, 4.999999999999999]
c1_y = [4.999999999999998, 0.0, -0.0, 1.8394736842105257, 0.0]
c2_x = [-0.0, 0.0, 3.495000000000001]
c2_y =  [0.0, 1.839473684210528, 0.0]
getSharedBoundary6(c1_x,c1_y,c2_x,c2_y)
'''



        