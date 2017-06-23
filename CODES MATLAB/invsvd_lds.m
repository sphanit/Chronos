%#####################################################################################################
%# Function : Inverse of a matrix using SVD-LDS                                                      #   
%#                                                                                                   #       
%# Input(s)  : J(matrix), lambda(some positive scalar between (0,1] )                                               #
%#                                                                                                   #                
%# Ouptut(s) : matrix obtained by doing SVD-LDS on J                                                # 
%#                                                                                                   #       
%# Example: invsvd_lds(matrix,scalar)                                                           #   
%#                                                                                                   #       
%#                                                                                                   #               
%#                                                                                                   #                                       
%#####################################################################################################


function JI = invsvd_lds(J,lamda)

[U,d,V] = svd(J);
E = zeros(size(d,2), size(d,1));

l = min(size(d,1),size(d,2));

for i = 1:l
    if d(i,i)~=0
        E(i,i) = d(i,i)/(d(i,i)^2 + lamda^2);
    end
end


JI = V*E*(U');
end

