function h=forecast(e,t)
%FORECAST Tail freeze is an approximation, not knowledge of the final plateau.
h=min(1,max(e.floor,e.current+e.slope*min(max(t(:)',0),e.horizon)));
end
