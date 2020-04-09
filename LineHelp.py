import GraphDomain as gd

#the following 2 functions where taken from https://stackoverflow.com/questions/3838329/how-can-i-check-if-two-segments-intersect
#by user Grumdrig
#and subsequently greatly added to to provide aditional functionality

def ccw(A,B,C):
    return (C.y-A.y) * (B.x-A.x) > (B.y-A.y) * (C.x-A.x)


# Return true if line segments AB and CD intersect
def intersect(A,B,C,D):

    
    
    
    s1 = 0 if (B.x==A.x) else (B.y-A.y)/(B.x-A.x)
    s2 = 0 if(D.x == C.x) else (D.y-C.y)/(D.x-C.x)
    
    b1 = A.y - (A.x*s1)
    b2 = C.y - (C.x*s2)

    if ((C.y == s1*C.x + b1) or (D.y == s1*D.x + b1)  or (A.y == s2*A.x + b2) or (B.y == s2*B.x + b2)):
        #print("on line")
        return False
    
    return (ccw(A,C,D) != ccw(B,C,D) and ccw(A,B,C) != ccw(A,B,D)) #or (C.y == s1*C.x + b1) or (D.y == s1*D.x + b1)  or (A.y == s2*A.x + b2) or (B.y == s2*B.x + b2)


      