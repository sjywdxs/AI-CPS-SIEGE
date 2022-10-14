function [rewardIndex, rewardInterval] = RewardClustering(reward, minDist, minNum, maxDist)
% Cluster a reward array with conditions on minimum distance and minimum number
%
% Input: 
%   reward: 1-D array contains reward value of each state
%   minDist: minimum value between slice. lb of current slice - ub of previouse slice
%   minNum: minimum number of states per slice
%
% Output:
%   rewardIndex: index array of each reward
%   rewardInterval: interval[lb, ub] w.r.t. each index

    rewardIndex = zeros(length(reward),1);
    rewardInterval = {1, length(reward)};
    index = 1;
    lb = min(reward);
    ub = max(reward);

    while ~all(rewardIndex)         % loop until each reward is allocated an index
        interval = BinarySplit(reward, lb, ub, minDist, minNum,maxDist);
        lb = interval(2);           % sliding window, move lb to ub

        if lb < ub
           % assign index to rewards within current interval
            rewardIndex(reward>=interval(1) & reward<interval(2)) = index;
            rewardInterval{index} = interval;
            index = index+1;
        else
            % if sliding window reaches the end
            rewardIndex(reward>=interval(1)) = index;
            rewardInterval{index} = interval;
        end

    end


end
