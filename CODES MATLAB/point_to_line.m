%#####################################################################################################
%# Function : To calculate the  perpendicular distance between the point and the line defined by     #   
%#             the two vectors v1,v2                                                                 #       
%# Input(s)  : pt(point),v1 & v2 - the 2 vectors on the line                                         #
%#                                          _                                                        #                
%# Ouptut(s) : d(distance between the line and the point)                                            # 
%#                                                                                                   #       
%# Example: point_to_line([a1,b1,c1],[x1,y1,z1],[x2,y2,z2])                                          #   
%#                                                                                                   #       
%#                                                                                                   #               
%#                                                                                                   #                                       
%#####################################################################################################


function d = point_to_line(pt, v1, v2)
      a = v1 - v2;
      b = pt - v2;
      
      %d = norm(cross(v1-pt,v2-pt)) / norm(a);
     c1 = dot(a,b);
     if ( c1 <= 0 )
      d = norm(pt-v2);
      return;
     end
     
     c2 = dot(a,a);
     if ( c2 <= c1 )
         d= norm(pt-v1);
 
         return;
     end
          
     temp = c1 / c2;
     p = v1 + temp * a;
     d = norm(pt-p);

end