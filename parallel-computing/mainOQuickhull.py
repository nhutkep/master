import matplotlib.pyplot as plt
import csv
import time
import concurrent.futures as cf

# Read input points.
input_points = []
dataset = []

with open("200.csv") as f:
    csv_reader = csv.reader(f, delimiter=",")
    for row in csv_reader:
        input_points = input_points + [[float(row[0]), float(row[1])]]

def find_extreme_points(points):
    maxY = points[0][1]
    minY = points[0][1]
    maxX = points[0][0]
    minX = points[0][0]
    
    leftPoints = []
    rightPoints = []
    topPoints = []
    bottomPoints = []
    
    for point in points:
        if point[0] < minX: minX = point[0] #leftmost point
        if point[0] > maxX: maxX = point[0] #rightmost point
        if point[1] < minY: minY = point[1] #lowest point
        if point[1] > maxY: maxY = point[1] #hightest point


    for point in points:
        if point[0] == minX: leftPoints.append(point) #add the points having minimal x-coordinate in lefPoints list
        if point[0] == maxX: rightPoints.append(point) #add the points having maxi mal x-coordinate in lefPoints list
        if point[1] == minY: bottomPoints.append(point) #add the points having minimal y-coordinate in lefPoints list
        if point[1] == maxY: topPoints.append(point) #add the points having maximal y-coordinate in lefPoints list

    # Sort the elements of the lists: topPoints, bottomPoints, rightPoints, and leftPoints and give the starting and ending points of each list

    if len(topPoints) == 1:
        top = (topPoints[0],)
    else:
        topPoints = sorted(topPoints, key = lambda x : x[0])
        top = (topPoints[0], topPoints[len(topPoints)-1])

    # bottom
    if len(bottomPoints) == 1:
        bottom = (bottomPoints[0],)
    else:
        bottomPoints = sorted(bottomPoints, key = lambda x : -x[0])
        bottom = (bottomPoints[0], bottomPoints[len(bottomPoints)-1])

    # right
    if len(rightPoints) == 1:
        right = (rightPoints[0],)
    else:
        rightPoints = sorted(rightPoints, key = lambda x : -x[1])
        right = (rightPoints[0], rightPoints[len(rightPoints)-1])

    # left
    if len(leftPoints) == 1:
        left = (leftPoints[0],)
    else:
        leftPoints = sorted(leftPoints, key = lambda x : x[1])
        left = (leftPoints[0], leftPoints[len(leftPoints)-1])

    return top,left,bottom,right

def partition(set, q, qq, quad):
    '''
    Tính bình phương khoảng cách từ điểm p tới đỉnh v(qq2[0], q2[1])
    '''
    def add(p):
        return (p[0] - qq[0])*(p[0] - qq[0]) + (p[1] - q[1])*(p[1] - q[1])
    '''
    Sắp xếp theo thứ tự giảm dần các khoảng cách từ p với v. New point là điểm đầu tiên trong dãy.    
    '''
    sort_set_Dist = sorted(set, key = add, reverse=True)

    '''
    max radius
    '''
    new_point = sort_set_Dist[0]
    
    new_set1 = []
    new_set2 = []

    if (quad == 1):
        for p in set:
            if p[0] > new_point[0]:
                new_set1.append(p)
            elif p[1] > new_point[1]:
                new_set2.append(p)
    elif quad == 2:
        for p in set:
            if p[0] < new_point[0]:
                new_set1.append(p)
            elif p[1] > new_point[1]:
                new_set2.append(p)
    elif quad == 3:
        for p in set:
            if p[0] < new_point[0]:
                new_set1.append(p)
            elif p[1] < new_point[1]:
                new_set2.append(p)
    elif quad == 4:
        for p in set:
            if p[0] > new_point[0]:
                new_set1.append(p)
            elif p[1] < new_point[1]:
                new_set2.append(p)
    return new_point, new_set1, new_set2
    
def find_o_hull(set, q, qq, quad):
    if len(set) == 0:
        if quad == 1 or quad == 3:
            return [qq]
        else:
            return [q]
    
    new_point, new_set1, new_set2 = partition(set, q, qq, quad)

    if quad == 1 or quad == 3:
        return [q] + find_o_hull(new_set1, q, new_point, quad) + find_o_hull(new_set2, new_point, qq, quad)
    else:
        return [qq] + find_o_hull(new_set2, new_point, qq, quad) + find_o_hull(new_set1, q, new_point, quad)

def findOCH(set, q, qq, quad):
    arranged_points = []
    if len(set) <= 1000:
        arranged_points = find_o_hull(set, q, qq, quad)
    else:
        new_point, new_set1, new_set2 = partition(set, q, qq, quad)
        quads = [quad, quad]
        results = []
        if quad == 1 or quad == 3:
            sets = [new_set1, new_set2]
            qs = [q, new_point]
            qqs = [new_point, qq]
        else:
            sets = [new_set2, new_set1]
            qs = [new_point, q]
            qqs = [qq, new_point]
            
        with cf.ThreadPoolExecutor(max_workers=2) as executor:
            results = executor.map(findOCH, sets, qs, qqs, quads)

            for result in results:
                arranged_points += result
    return arranged_points

def findOrthogonalConvexHull_serial(points):
    # Find 8 points
    top, left, bottom, right = find_extreme_points(points)
    if len(top) == 1:
        q1 = qq4 = top[0]
    else:
        q1 = top[0]
        qq4 = top[1]
    q4 = right[0]
    if len(right) == 1:
        qq3 = right[0]
    else:
        qq3 = right[1]
    q3 = bottom[0]
    if len(bottom) == 1:
        qq2 = bottom[0]
    else:
        qq2 = bottom[1]
    q2 = left[0]
    if len(left) == 1:
        qq1 = left[0]
    else:
        qq1 = left[1]

    # Separate to 4 sets of points
    set1 = []
    set2 = []
    set3 = []
    set4 = []
    
    # for p in points:
    #     if p[0] <= q1[0] and p[1] >= qq1[1]:
    #         set1.append(p)
    #     if p[0] <= qq2[0] and p[1] <= q2[1]:
    #         set2.append(p)
    #     if p[0] >= q3[0] and p[1] <= qq3[1]:
    #         set3.append(p)
    #     if p[0] >= qq4[0] and p[1] >= q4[1]:
    #         set4.append(p)
    for p in points:
        if p[0] < q1[0] and p[1] > qq1[1]:
            set1.append(p)
        if p[0] < qq2[0] and p[1] < q2[1]:
            set2.append(p)
        if p[0] > q3[0] and p[1] < qq3[1]:
            set3.append(p)
        if p[0] > qq4[0] and p[1] > q4[1]:
            set4.append(p) 

    arranged_points = []

    arranged_points = arranged_points + [q1] + find_o_hull(set1, qq1, q1, 2) + [qq1]
    arranged_points = arranged_points + [q2] + find_o_hull(set2, q2, qq2, 3) + [qq2]
    arranged_points = arranged_points + [q3] + find_o_hull(set3, qq3, q3, 4) + [qq3]
    arranged_points = arranged_points + [q4] + find_o_hull(set4, q4, qq4, 1) + [qq4]

    return arranged_points

def findOrthogonalConvexHull_multhr(points):
    # Find 8 points
    top, left, bottom, right = find_extreme_points(points)

    if len(top) == 1:
        q1 = qq4 = top[0]
    else:
        q1 = top[0]
        qq4 = top[1]
    q4 = right[0]
    if len(right) == 1:
        qq3 = right[0]
    else:
        qq3 = right[1]
    q3 = bottom[0]
    if len(bottom) == 1:
        qq2 = bottom[0]
    else:
        qq2 = bottom[1]
    q2 = left[0]
    if len(left) == 1:
        qq1 = left[0]
    else:
        qq1 = left[1]

    # Separate to 4 sets of points
    set1 = []
    set2 = []
    set3 = []
    set4 = []

    # for p in points:
    #     if p[0] <= q1[0] and p[1] >= qq1[1]:
    #         set1.append(p)
    #     if p[0] <= qq2[0] and p[1] <= q2[1]:
    #         set2.append(p)
    #     if p[0] >= q3[0] and p[1] <= qq3[1]:
    #         set3.append(p)
    #     if p[0] >= qq4[0] and p[1] >= q4[1]:
    #         set4.append(p)

    for p in points:
        if p[0] < q1[0] and p[1] > qq1[1]:
            set1.append(p)
        if p[0] < qq2[0] and p[1] < q2[1]:
            set2.append(p)
        if p[0] > q3[0] and p[1] < qq3[1]:
            set3.append(p)
        if p[0] > qq4[0] and p[1] > q4[1]:
            set4.append(p) 

    arranged_points = []
    sets = [set1, set2, set3, set4]
    qs = [qq1, q2, qq3, q4]
    qqs = [q1, qq2, q3, qq4]
    quads = [2, 3, 4, 1]

    with cf.ThreadPoolExecutor(max_workers=4) as executor:
        results = executor.map(findOCH, sets, qs, qqs, quads)

        for result in results:
            arranged_points += result

    return arranged_points

def findOrthogonalConvexHull_mulprc(points):
    # Find 8 points
    top, left, bottom, right = find_extreme_points(points)

    if len(top) == 1:
        q1 = qq4 = top[0]
    else:
        q1 = top[0]
        qq4 = top[1]
    q4 = right[0]
    if len(right) == 1:
        qq3 = right[0]
    else:
        qq3 = right[1]
    q3 = bottom[0]
    if len(bottom) == 1:
        qq2 = bottom[0]
    else:
        qq2 = bottom[1]
    q2 = left[0]
    if len(left) == 1:
        qq1 = left[0]
    else:
        qq1 = left[1]

    # Separate to 4 sets of points
    set1 = []
    set2 = []
    set3 = []
    set4 = []

    for p in points:
        if p[0] <= q1[0] and p[1] >= qq1[1]:
            set1.append(p)
        if p[0] <= qq2[0] and p[1] <= q2[1]:
            set2.append(p)
        if p[0] >= q3[0] and p[1] <= qq3[1]:
            set3.append(p)
        if p[0] >= qq4[0] and p[1] >= q4[1]:
            set4.append(p)

    arranged_points = []
    sets = [set1, set2, set3, set4]
    qs = [qq1, q2, qq3, q4]
    qqs = [q1, qq2, q3, qq4]
    quads = [2, 3, 4, 1]

    with cf.ProcessPoolExecutor(max_workers=5) as executor:
        results = executor.map(find_o_hull, sets, qs, qqs, quads)

        for result in results:
            arranged_points += result

    return arranged_points

def plotOCH(points):
    S = []
    n = len(points)
    for i in range(0, n-1):

        if points[i+1][0] > points[i][0] and points[i+1][1] > points[i][1]:
            
            p3 = [points[i][0], points[i+1][1]]
            
            S.append([i+1, p3])

        elif points[i+1][0] > points[i][0] and points[i+1][1] < points[i][1]:
            
            p3 = [points[i+1][0], points[i][1]]
            
            S.append([i+1, p3])

        elif points[i+1][0] < points[i][0] and points[i+1][1] < points[i][1]:

            p3 = [points[i][0], points[i+1][1]]

            S.append([i+1, p3])

        elif points[i+1][0] < points[i][0] and points[i+1][1] > points[i][1]:
            
            p3 = [points[i+1][0], points[i][1]]
            
            S.append([i+1, p3])

    #print("list index and points to be adding: ", S)

    for i in range(len(S)):
        points.insert(S[i][0]+i, S[i][1])
    points.append(points[0])
    ############################################################
    # Draw lines
    fig, ax = plt.subplots()

    # Input points in blue
    ax.plot([x[0] for x in input_points], [x[1] for x in input_points], 'r.')

    # Orthogonal convex hull points in red
    ax.plot([x[0] for x in points], [x[1] for x in points], 'b-')
    #ax.plot([list(points[0])[0], list(points[-1])[0]], [list(points[0])[1], list(points[-1])[1]],'r-')

    plt.ylabel('Y')
    plt.xlabel('X')
    plt.show()

if __name__ == '__main__':
    # Time measurement starts here
    start = time.perf_counter()
    # Find orthogonal points sequence (arranged to order)
    points = findOrthogonalConvexHull_serial(input_points)
    # Time measurement ends here
    finish = time.perf_counter()
    print(f"Algorithm running time: {round(finish - start, 5)} seconds.")
    plotOCH(points)
    
    # Time measurement starts here
    start = time.perf_counter()
    # Find orthogonal points sequence (arranged to order)
    points = findOrthogonalConvexHull_multhr(input_points)
    # Time measurement ends here
    finish = time.perf_counter()
    print(f"Algorithm running time: {round(finish - start, 5)} seconds.")
    plotOCH(points)