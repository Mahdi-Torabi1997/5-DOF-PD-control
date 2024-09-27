% Updates the the following properties of the pancake struct
%  - Sets current pancake as next pancake
%  - Sets current moving status as false
%  - Sets current pancake task
%  - Sets the next position where the pancake should start moving
%  - Sets the next position where the pancake should stop moving

function pancake = update_pancake(pancake, reset)
    arguments
        pancake (1,1) struct
        reset (1,:) string = "false"
    end
    
    pancake.move = false;
    
    if any(reset == {'reset', 'true'})
        pancake.num = 1;
        pancake.task = 'flip';
        pancake.nextSet = false;
    else
        if mod(pancake.num,3) == 0
            if pancake.task == "flip"
                pancake.task = "serve";
                pancake.num = pancake.num - 2;
            elseif pancake.task == "serve"
                pancake.task = "flip";
                pancake.num = pancake.num + 1;
                pancake.nextSet = true;
            end
        else
            pancake.num = pancake.num + 1;
        end
    end
    
    pancake.startPos = update_start_pos(pancake);
    pancake.endPos = update_end_pos(pancake);
    
    function pos = update_start_pos(pancake)
        num = pancake.num;
        pos = [ 380                  ; 
                355+200*mod(num-1,3) ; 
                -10                 ];
    end

    function pos = update_end_pos(pancake)
        num = pancake.num;
        task = pancake.task;
        if task == "flip"
            pos = [ 380                  ; 
                    355+200*mod(num-1,3) ; 
                    0                   ];
        elseif task == "serve"
            pos = [ 380        ; 
                    80         ; 
                    10*num-50 ];
        end
    end
end


