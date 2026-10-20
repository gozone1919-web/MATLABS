classdef MTISync < double
    enumeration
    	% all of the items must be in the specific order
    	Output_DOut0 (0),
		Output_Inverted_DOut0 (1),
    	Output_Sample_Clock (2),
    	Output_Start_Trigger (3),
    	External_Sample_Clock (4),
    	External_Start_Trigger (5),
        External_Frame_Trigger (6),
    end
end
