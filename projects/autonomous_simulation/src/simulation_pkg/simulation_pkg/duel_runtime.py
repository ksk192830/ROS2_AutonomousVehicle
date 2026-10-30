"""Two real vehicles, isolated controls and no competitor-to-competitor contact."""
import os
import tempfile
import xml.etree.ElementTree as ET


def set_text(parent, tag, value):
    child = parent.find(tag)
    if child is None:
        child = ET.SubElement(parent, tag)
    child.text = str(value)
    return child


def duel_vehicle(source, *, ai=False, controllers='', camera_filter=''):
    tree = ET.parse(source)
    model = tree.getroot().find('model')
    # ODE only creates a pair when the collision masks overlap. Both masks
    # overlap with the road and obstacles (0xffff), but not with each other.
    for collision in model.findall('.//collision'):
        surface = collision.find('surface')
        if surface is None:
            surface = ET.SubElement(collision, 'surface')
        contact = surface.find('contact')
        if contact is None:
            contact = ET.SubElement(surface, 'contact')
        set_text(contact, 'collide_bitmask', '2' if ai else '1')
    if ai:
        model.set('name', 'ai_rival')
        for visual in model.findall('.//visual'):
            set_text(visual, 'transparency', '.35')
        body = model.find(".//visual[@name='body_visual']")
        material = ET.SubElement(body, 'material')
        set_text(material, 'ambient', '.1 .85 .8 1')
        set_text(material, 'diffuse', '.1 .85 .8 1')
        for sensor in model.findall('.//sensor'):
            if sensor.get('name') == 'cam0':
                rate = sensor.find("plugin[@name='exhibition_front_rate']")
                if rate is not None:
                    rate.set('name', 'exhibition_opponent_rate')
                    set_text(rate, 'rate_topic', '/exhibition/camera_rate/opponent')
                plugin = sensor.find('plugin')
                ros = plugin.find('ros')
                if ros is None: ros = ET.SubElement(plugin, 'ros')
                set_text(ros, 'namespace', '/ai')
                ET.SubElement(ros, 'remapping').text = 'cam0/image_raw:=/ai/cam0/image_raw'
                ET.SubElement(ros, 'remapping').text = 'cam0/camera_info:=/ai/cam0/camera_info'
                set_text(plugin, 'cameraName', 'ai/cam0')
                set_text(plugin, 'imageTopicName', '/ai/cam0/image_raw')
                set_text(plugin, 'cameraInfoTopicName', '/ai/cam0/camera_info')
                ET.SubElement(sensor, 'plugin', name='hide_competitor', filename=camera_filter)
            elif sensor.get('type') == 'contact':
                set_text(sensor.find('plugin/ros'), 'namespace', '/ai/exhibition')
            else:
                # A second rear camera/lidar has no role in this exhibit.
                for parent in model.iter():
                    if sensor in list(parent): parent.remove(sensor); break
        plugin = model.find("plugin[@name='gazebo_ros2_control']")
        ros = ET.SubElement(plugin, 'ros')
        set_text(ros, 'namespace', '/ai')
        set_text(plugin, 'robot_param_node', '/ai/robot_state_publisher')
        set_text(plugin, 'parameters', controllers)
    fd, path = tempfile.mkstemp(prefix='duel_ai_' if ai else 'duel_player_', suffix='.sdf')
    os.close(fd)
    tree.write(path, encoding='utf-8', xml_declaration=False)
    return path
