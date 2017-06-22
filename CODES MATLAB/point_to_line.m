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