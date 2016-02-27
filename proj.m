function PT2 = proj(point, line)

vx = line(:, 3);
vy = line(:, 4);

dx = point(:,1) - line(:,1);
dy = point(:,2) - line(:,2);

px = -1*point(1)*(line(3) - line(1)) - point(2)*(line(4) - line(2));
py = -1*line(2)*(line(3) - line(1)) + line(1)*(line(4) - line(2));


rhs =-1*[px; py];
lhs(1,1) = (line(3) -line(1));
lhs(1,2) = (line(4) - line(2));
lhs(2,1) = (line(2) - line(4));
lhs(2,2) = (line(3)-line(1));

PT2 = (lhs)\rhs;
end