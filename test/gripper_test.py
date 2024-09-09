from fi_fsa import fi_fsa_v1
import time

if __name__ == '__main__':

    fi_fsa_v1.set_position_control("192.168.137.17", 0.0)
    fi_fsa_v1.set_enable("192.168.137.17")
    fi_fsa_v1.set_mode_of_operation("192.168.137.17", fi_fsa_v1.FSAModeOfOperation.POSITION_CONTROL)

    fi_fsa_v1.set_position_control("192.168.137.37", 0.0)
    fi_fsa_v1.set_enable("192.168.137.37")
    fi_fsa_v1.set_mode_of_operation("192.168.137.37", fi_fsa_v1.FSAModeOfOperation.POSITION_CONTROL)
    
    time.sleep(5)
    print("left 60!")
    fi_fsa_v1.fast_set_position_control("192.168.137.17", 60)
    fsa_state = fi_fsa_v1.get_state("192.168.137.17")
    print("State = %d" % fsa_state)

    time.sleep(2)
    print("left 20!")
    fi_fsa_v1.fast_set_position_control("192.168.137.17", 20)
    fsa_state = fi_fsa_v1.get_state("192.168.137.17")
    print("State = %d" % fsa_state)

    time.sleep(2)
    print("right 60!")
    fi_fsa_v1.fast_set_position_control("192.168.137.37", 30)
    fsa_state = fi_fsa_v1.get_state("192.168.137.17")
    print("State = %d" % fsa_state)

    time.sleep(2)
    print("right 20!")
    fi_fsa_v1.fast_set_position_control("192.168.137.37", 0)
    fsa_state = fi_fsa_v1.get_state("192.168.137.17")
    print("State = %d" % fsa_state)

    time.sleep(2)
    
    print("complete!")