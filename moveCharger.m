function new_charger_pos = moveCharger(charger_pos, target_pos, charger_speed)

% Move charger toward target position
direction = target_pos - charger_pos;
distance = norm(direction);
if distance > 0
    direction = direction / distance;
end
move_dist = min(distance, charger_speed);
new_charger_pos = charger_pos + direction * move_dist;

end