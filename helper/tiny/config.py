from os import getcwd

class Config(object):
    all_motions = ['backflip_scaled', 'cartwheel_scaled', 'crawl_scaled', 'dance_a_scaled', 'dance_b_scaled', 'getup_facedown_scaled'
                   'getup_faceup_scaled', 'jump_scaled', 'kick_scaled', 'punch_scaled', 'roll_scaled', 'run_scaled', 'spin_scaled', 'spinkick_scaled',
                   'walk_scaled']
    curr_path = getcwd()
    motion = 'dance_b_scaled'
    env_name = "subject_scaled_run_converted"

    motion_folder = '/helper/motions'
    xml_folder = '/helper/subject_scaled_run_converted'

    mocap_path = "%s%s/humanoid3d_%s.txt"%(curr_path, motion_folder, motion)
    xml_path = "%s%s/%s.xml"%(curr_path, xml_folder, env_name)

  