function interval = BinarySplit(reward, lb, ub, minDist, minNum)
% Split a reward array and returen the lower & upper bound of the interval
%
% Input: 
%   reward: 1-D array contains reward value of each state
%   lb: lower bound, minimum value of the reward 
%   ub: upper bound, maximum value of the reward
%   minDist: minimum value between slice. lb of current slice - ub of previouse slice
%   minNum: minimum number of states per slice
%
% Output:
%   interval: lb and up of valid interval

        ubNew = (ub+lb)/2;
        numValidState = sum(reward>=lb & reward<ubNew);

        if ubNew-lb > minDist && numValidState > minNum
            interval = BinarySplit(reward, lb, ubNew, minDist, minNum);
        else
            interval = [lb, ub];
        end

end
