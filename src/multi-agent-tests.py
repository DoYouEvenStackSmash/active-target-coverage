def multi_agent_mouse_test(screen, environment):
    pt = None
    last_pt = None
    while 1:
        for event in pygame.event.get():
            if event.type == pygame.MOUSEBUTTONDOWN:
                for _id in environment.agents:
                    sensing_agent = environment.agents[_id]
                    e = sensing_agent.export_tracks()
                    f = open(f"{_id}_out.json", "w")
                    f.write(json.dumps(e, indent=2))
                    f.close()
                sys.exit()
            pt = pygame.mouse.get_pos()
            if last_pt == pt:
                continue

            last_pt = pt
            pafn.clear_frame(screen)

            for k in environment.agents:
                agent_update(environment.agents[k])

                curr_pt, pred_pt = environment.agents[k].estimate_next_detection()

                if len(pred_pt):
                    pafn.frame_draw_dot(screen, curr_pt, pafn.colors["tangerine"])
                    pafn.frame_draw_dot(screen, pred_pt, pafn.colors["yellow"])
                    pafn.frame_draw_line(
                        screen, (curr_pt, pred_pt), pafn.colors["white"]
                    )

            for k in environment.agents:
                sensing_agent = environment.agents[k]
                draw_sensing_agent(screen, sensing_agent)
            environment.targets[0].origin = pt
            pafn.frame_draw_dot(screen, pt, pafn.colors["lawngreen"])
            environment.visible_targets()
            pygame.display.update()
