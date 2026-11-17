"""Physical walls share exactly the rectangles used by the navigation map."""
from .push_planning import walls
from .scene import box_link


def setup_scene(env, p, box, output, c):
    from igibson.objects.articulated_object import ArticulatedObject
    env.land(env.robots[0], [*c['start'], .1], [0, 0, 0])
    p.resetBasePositionAndOrientation(box, [*c['box_position'], c['box_size'][2]/2+.01], [0, 0, 0, 1])
    objects = []
    ids = set()
    for i, (x0, y0, x1, y1) in enumerate(walls(c)):
        path = output/f'wall_{i}.urdf'
        # Low walls expose the interaction to the fixed camera.
        path.write_text('<robot name="wall">'+box_link('base', ((x0+x1)/2, (y0+y1)/2, .3),
                        (x1-x0, y1-y0, .6))+'</robot>', encoding='utf-8')
        obj = ArticulatedObject(str(path))
        env.simulator.import_object(obj)
        objects.append(obj)
        ids.update(obj.get_body_ids())
    env.push_walls = objects
    return ids
