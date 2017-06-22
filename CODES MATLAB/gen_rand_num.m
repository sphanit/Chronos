function[R] = gen_rand_num(arr,n)
SET = arr ;
NSET = length(SET) ;

N = n ; % pick N numbers
i = ceil(NSET*rand(1,N)) ; % with repeat
R = SET(i) ;
end