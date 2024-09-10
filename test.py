import algo

boxes = [[0.0, 0.0, 1.0, 1.0], [0.5, 0.5, 1.5, 1.5]]
scores = [0.9, 0.8]
threshold = 0.1

result = algo.nms_rotated(boxes, scores, threshold)
print(result)
