%#####################################################################################################
%# Function : generating a sequence of random numbers lying in the range specified by arr            #                                      #   
%#                                                                                                   #       
%# Input(s)  : arr(range of numbers), n(number of random numbers to be generated)                    #
%#                                                                                                   #                
%# Ouptut(s) : sequence of random numbers lying in the required range                                # 
%#                                                                                                   #       
%# Example: Hom_Trans(-90, 10, 5, 90)                                                                #   
%#                                                                                                   #       
%#                                                                                                   #               
%#                                                                                                   #                                       
%#####################################################################################################


function[R] = gen_rand_num(arr,n)
SET = arr ;
NSET = length(SET) ;

N = n ; % pick N numbers
i = ceil(NSET*rand(1,N)) ; % with repeat
R = SET(i) ;
end