function draw_formation(Fleet)
    if size(Fleet, 1) >= 4
        plot([Fleet(1,1), Fleet(2,1)], [Fleet(1,2), Fleet(2,2)], '--k');
        plot([Fleet(2,1), Fleet(4,1)], [Fleet(2,2), Fleet(4,2)], '--k');
        plot([Fleet(4,1), Fleet(3,1)], [Fleet(4,2), Fleet(3,2)], '--k');
        plot([Fleet(3,1), Fleet(1,1)], [Fleet(3,2), Fleet(1,2)], '--k');
    end
end
