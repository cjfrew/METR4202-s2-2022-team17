from turtle import color


class GetColor(sm.State):
    def __init__(self):
        self.same_color_tally = 0
        self.last_seen_color = 'NONE'
        self.block_color = 'NONE' # NONE RED BLUE GREEN YELLOW

    def update_color(self, msg):
        if self.last_seen_color == msg.data:
            self.same_color_tally += 1
        else:
            self.same_color_tally = 0
        
        if self.same_color_tally >= 10:
            self.block_color = msg.data
        else:
            self.block_color = 'NONE'

    def excecute(self, ud):
        while self.block_color == 'NONE':
            rp.sleep(0.2)

        color_output_map = {'RED':'red',
                            'BLUE': 'blue',
                            'GREEN':'green',
                            'YELLOW':'yellow'}
        return color_output_map[self.block_color]
        