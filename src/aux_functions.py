#!/usr/bin/python3
import cv2
import numpy as np
import magic
import re
# STANDARD_COLORS = [
#		 'AliceBlue', 'Chartreuse', 'Aqua', 'Aquamarine', 'Azure', 'Beige', 'Bisque',
#		 'BlanchedAlmond', 'BlueViolet', 'BurlyWood', 'CadetBlue', 'AntiqueWhite',
#		 'Chocolate', 'Coral', 'CornflowerBlue', 'Cornsilk', 'Crimson', 'Cyan',
#		 'DarkCyan', 'DarkGoldenRod', 'DarkGrey', 'DarkKhaki', 'DarkOrange',
#		 'DarkOrchid', 'DarkSalmon', 'DarkSeaGreen', 'DarkTurquoise', 'DarkViolet',
#		 'DeepPink', 'DeepSkyBlue', 'DodgerBlue', 'FireBrick', 'FloralWhite',
#		 'ForestGreen', 'Fuchsia', 'Gainsboro', 'GhostWhite', 'Gold', 'GoldenRod',
#		 'Salmon', 'Tan', 'HoneyDew', 'HotPink', 'IndianRed', 'Ivory', 'Khaki',
#		 'Lavender', 'LavenderBlush', 'LawnGreen', 'LemonChiffon', 'LightBlue',
#		 'LightCoral', 'LightCyan', 'LightGoldenRodYellow', 'LightGray', 'LightGrey',
#		 'LightGreen', 'LightPink', 'LightSalmon', 'LightSeaGreen', 'LightSkyBlue',
#		 'LightSlateGray', 'LightSlateGrey', 'LightSteelBlue', 'LightYellow', 'Lime',
#		 'LimeGreen', 'Linen', 'Magenta', 'MediumAquaMarine', 'MediumOrchid',
#		 'MediumPurple', 'MediumSeaGreen', 'MediumSlateBlue', 'MediumSpringGreen',
#		 'MediumTurquoise', 'MediumVioletRed', 'MintCream', 'MistyRose', 'Moccasin',
#		 'NavajoWhite', 'OldLace', 'Olive', 'OliveDrab', 'Orange', 'OrangeRed',
#		 'Orchid', 'PaleGoldenRod', 'PaleGreen', 'PaleTurquoise', 'PaleVioletRed',
#		 'PapayaWhip', 'PeachPuff', 'Peru', 'Pink', 'Plum', 'PowderBlue', 'Purple',
#		 'Red', 'RosyBrown', 'RoyalBlue', 'SaddleBrown', 'Green', 'SandyBrown',
#		 'SeaGreen', 'SeaShell', 'Sienna', 'Silver', 'SkyBlue', 'SlateBlue',
#		 'SlateGray', 'SlateGrey', 'Snow', 'SpringGreen', 'SteelBlue', 'GreenYellow',
#		 'Teal', 'Thistle', 'Tomato', 'Turquoise', 'Violet', 'Wheat', 'White',
#		 'WhiteSmoke', 'Yellow', 'YellowGreen'
# ]


class ArtFxns:
	
	def draw_circle(img1, ybbox, color=(255,0,0), radius=1, thickness=5):
		'''
		Draws a circle in the center of a bounding box
		'''
		x,y = ybbox.get_center_coord()
		r,g,b = color
		cv2.circle(img1, (int(x),int(y)), radius, (int(r),int(g),int(b)), thickness)
		
	
	def draw_rectangle(img1,ybbox,color=(255,255,0),thickness=2):
		'''
		Draws the bounding box of ybbox in color on img1
		'''
		corners = ybbox.get_corner_coords()
		x1,y1 = corners[0]
		pt1 = (int(x1),int(y1))
		x2,y2 = corners[1]
		pt2 = (int(x2),int(y2))
		cv2.rectangle(img1, pt1, pt2, color, thickness)
		

	def draw_line(img1, ybbox,color=(255,255,255),thickness=4):
		'''
		Draws a line connecting the center of a bounding box to the center
		of its successor, or no line if last in the track
		'''
		yb2 = ybbox.next
		if yb2 == None:
			# ArtFxns.draw_circle(img1,ybbox,color, radius=8, thickness=2)
			return
		x1,y1 = ybbox.get_center_coord()
		pt1 = (int(x1),int(y1))
		x2,y2 = yb2.get_center_coord()
		pt2 = (int(x2),int(y2))
		cv2.line(img1, pt1, pt2, color, thickness)

	def draw_text(img1, ybbox, color = (255,255,255),offt=10):
		'''
		Draws the ybbox parent track id on an entity
		'''
		pt = ybbox.get_center_coord()
		font = cv2.FONT_HERSHEY_SIMPLEX
		pt = (int(pt[0]),int(pt[1]))
		cv2.putText(img1, str(ybbox.parent_track), pt, font, 2, color, 4, cv2.LINE_AA)

	def draw_label(img1, ybbox, label, color = (255,255,255),offt=10):
		'''
		Draw a text label on the entity
		'''
		pt = ybbox.get_center_coord()
		font = cv2.FONT_HERSHEY_SIMPLEX
		pt = (int(pt[0]) - offt * 2,int(pt[1] - offt))
		cv2.putText(img1, label, pt, font, 1, color, 4, cv2.LINE_AA)

class ImgFxns:
	'''
	Image transform helper functions
	'''
	def get_img_shape(valid_img_filename):
		print(valid_img_filename)
		magic_data = magic.from_file(str(valid_img_filename))
		print(magic_data)
		width, height = re.search('(\d+) x (\d+)', magic_data).groups()
		return int(width), int(height)

	def rotate_image_2(img1, image_center, angle):
		'''
		Theoretical rotation function
		Apparently cv2 does something with image scaling, which prevents this from 
		functioning properly.
		'''
		cy,cx = image_center
		
		rot_mat = cv2.getRotationMatrix2D(image_center, angle, 1.0)
		return cv2.warpAffine(img1,rot_mat, image_center,flags=cv2.INTER_LINEAR)

	def rotate_image(img1, image_center, angle):
		'''
		Rotate a single image
		2D rotation matrix
			|cos(theta),-sin(theta)| |x| = R(theta) |x|
			|sin(theta), cos(theta)| |y|						|y|
		'''
		w, h = image_center[0] * 2, image_center[1] * 2
		
		rot_mat = cv2.getRotationMatrix2D(image_center, angle, 1.0)
		
		abs_cos = abs(rot_mat[0,0])
		abs_sin = abs(rot_mat[0,1])
		
		b_w = int(w * abs_cos + h * abs_sin)
		b_h = int(w * abs_sin + h * abs_cos)
		# Translate to origin
		rot_mat[0,2] += b_w / 2 - image_center[0]
		rot_mat[1,2] += b_h / 2 - image_center[1]
		
		# perform rotation
		return cv2.warpAffine(img1,rot_mat, (b_w, b_h) ,flags=cv2.INTER_LINEAR)
	
	# rotate images, generate new ones
	def rotate_images(images, angle):
		'''
		Rotate all images and change their dimensions, LOCO format
		image = {	
			"id":int
			"file_name":string,
			"height":int,
			"width":int,
		}
		'''
		for i,img in enumerate(images):
			img1 = cv2.imread(f"{img['file_name']}")
			
			# perform rotation to match already rotated bounding boxes
			if angle != 0:
				img_center = (int(img['height'] / 2), int(img['width'] / 2))
				img1 = ImgFxns.rotate_image(img1, img_center, angle)
			
			# prepend rotated and angle prefix to file name
			fn = f"rotated_{angle}_{img['file_name'].split('/')[-1]}"
			
			# update image in list with new name
			img['file_name'] = fn
			w,h = img1.shape[1],img1.shape[0]
			img['height'] = h
			img['width'] = w
			
			# write file
			cv2.imwrite(f"{fn.split('/')[-1]}", img1)
		return images
	
	def reflect_image(img1, AXIS = 1):
		'''
		Reflect an image about a specified axis
		'''
		return cv2.flip(img1, AXIS)
	
	def reflect_images(images, AXIS):
		'''
		Reflect all images, do not change their dimensions, LOCO format
		image = {	
			"id":int
			"file_name":string,
			"height":int,
			"width":int,
		}
		'''
		for i, img in enumerate(images):
			img1 = cv2.imread(f"{img['file_name']}")
			
			# perform reflection to match rotated bounding boxes
			img1 = ImgFxns.reflect_image(img1, AXIS)
			
			# prepend reflected and axis prefix to file name
			dir_str = "x" if AXIS == 0 else "y"
			fn = f"reflected_{dir_str}_{img['file_name'].split('/')[-1]}"
			
			# update image in list with new name
			img['file_name'] = fn
			w,h = img1.shape[1],img1.shape[0]
			img['height'] = h
			img['width'] = w
			
			# write file
			cv2.imwrite(f"{fn.split('/')[-1]}",img1)
		return images

			

class MathFxns:
	'''
	Math helper functions
	'''
	def euclidean_dist(p1, p2):
		'''
		Calculates euclidean distance between two points
		Returns a scalar value
		'''
		return np.sqrt(np.square(p1[0] - p2[0]) + np.square(p1[1] - p2[1]))

SCSET = [
				"lightsalmon","salmon","darksalmon","lightcoral","indianred","crimson",
				"firebrick","red","darkred","coral","tomato","orangered","gold",
				"orange","darkorange","lightyellow","lemonchiffon","lightgoldenrodyellow","papayawhip","moccasin",
				"peachpuff","palegoldenrod","khaki","darkkhaki","yellow","lawngreen","chartreuse",
				"limegreen","lime","forestgreen","green","darkgreen","greenyellow","yellowgreen",
				"springgreen","mediumspringgreen","lightgreen","palegreen","darkseagreen","mediumseagreen","seagreen",
				"olive","darkolivegreen","olivedrab","lightcyan","cyan","aqua","aquamarine",
				"mediumaquamarine","paleturquoise","turquoise","mediumturquoise","darkturquoise","lightseagreen","cadetblue","darkcyan",
				"teal","powderblue","lightblue","lightskyblue","skyblue","deepskyblue","lightsteelblue",
				"dodgerblue","cornflowerblue","steelblue","royalblue","blue","mediumblue","darkblue",
				"navy","midnightblue","mediumslateblue","slateblue","darkslateblue","lavender","thistle",
				"plum","violet","orchid","fuchsia","magenta","mediumorchid","mediumpurple",
				"blueviolet","darkviolet","darkorchid","darkmagenta","purple","indigo","pink",
				"lightpink","hotpink","deeppink","palevioletred","mediumvioletred","white","snow",
				"honeydew","mintcream","azure","aliceblue","ghostwhite","whitesmoke","seashell",
				"beige","oldlace","floralwhite","ivory","antiquewhite","linen","lavenderblush",
				"mistyrose","gainsboro","lightgray","silver","darkgray","gray","dimgray",
				"lightslategray","slategray","darkslategray","black","cornsilk","blanchedalmond","bisque",
				"navajowhite","wheat","burlywood","tan","rosybrown","sandybrown","goldenrod",
				"peru","chocolate","saddlebrown","sienna","brown","maroon"
				]

STANDARD_COLORS = {
 	"lightsalmon":(255,160,122),
 	"salmon":(250,128,114),
 	"darksalmon":(233,150,122),
 	"lightcoral":(240,128,128),
 	"indianred":(205,92,92),
 	"crimson":(220,20,60),
 	"firebrick":(178,34,34),
 	"red":(255,0,0),
 	"darkred":(139,0,0),
 	"coral":(255,127,80),
 	"tomato":(255,99,71),
 	"orangered":(255,69,0),
 	"gold":(255,215,0),
 	"orange":(255,165,0),
 	"darkorange":(255,140,0),
 	"lightyellow":(255,255,224),
 	"lemonchiffon":(255,250,205),
 	"lightgoldenrodyellow":(250,250,210),
 	"papayawhip":(255,239,213),
 	"moccasin":(255,228,181),
 	"peachpuff":(255,218,185),
 	"palegoldenrod":(238,232,170),
 	"khaki":(240,230,140),
 	"darkkhaki":(189,183,107),
 	"yellow":(255,255,0),
 	"lawngreen":(124,252,0),
 	"chartreuse":(127,255,0),
 	"limegreen":(50,205,50),
 	"lime":(0,255,0),
 	"forestgreen":(34,139,34),
 	"green":(0,128,0),
 	"darkgreen":(0,100,0),
 	"greenyellow":(173,255,47),
 	"yellowgreen":(154,205,50),
 	"springgreen":(0,255,127),
 	"mediumspringgreen":(0,250,154),
 	"lightgreen":(144,238,144),
 	"palegreen":(152,251,152),
 	"darkseagreen":(143,188,143),
 	"mediumseagreen":(60,179,113),
 	"seagreen":(46,139,87),
 	"olive":(128,128,0),
 	"darkolivegreen":(85,107,47),
 	"olivedrab":(107,142,35),
 	"lightcyan":(224,255,255),
 	"cyan":(0,255,255),
 	"aqua":(0,255,255),
 	"aquamarine":(127,255,212),
 	"mediumaquamarine":(102,205,170),
 	"paleturquoise":(175,238,238),
 	"turquoise":(64,224,208),
 	"mediumturquoise":(72,209,204),
 	"darkturquoise":(0,206,209),
 	"lightseagreen":(32,178,170),
 	"cadetblue":(95,158,160),
 	"darkcyan":(0,139,139),
 	"teal":(0,128,128),
 	"powderblue":(176,224,230),
 	"lightblue":(173,216,230),
 	"lightskyblue":(135,206,250),
 	"skyblue":(135,206,235),
 	"deepskyblue":(0,191,255),
 	"lightsteelblue":(176,196,222),
 	"dodgerblue":(30,144,255),
 	"cornflowerblue":(100,149,237),
 	"steelblue":(70,130,180),
 	"royalblue":(65,105,225),
 	"blue":(0,0,255),
 	"mediumblue":(0,0,205),
 	"darkblue":(0,0,139),
 	"navy":(0,0,128),
 	"midnightblue":(25,25,112),
 	"mediumslateblue":(123,104,238),
 	"slateblue":(106,90,205),
 	"darkslateblue":(72,61,139),
 	"lavender":(230,230,250),
 	"thistle":(216,191,216),
 	"plum":(221,160,221),
 	"violet":(238,130,238),
 	"orchid":(218,112,214),
 	"fuchsia":(255,0,255),
 	"magenta":(255,0,255),
 	"mediumorchid":(186,85,211),
 	"mediumpurple":(147,112,219),
 	"blueviolet":(138,43,226),
 	"darkviolet":(148,0,211),
 	"darkorchid":(153,50,204),
 	"darkmagenta":(139,0,139),
 	"purple":(128,0,128),
 	"indigo":(75,0,130),
 	"pink":(255,192,203),
 	"lightpink":(255,182,193),
 	"hotpink":(255,105,180),
 	"deeppink":(255,20,147),
 	"palevioletred":(219,112,147),
 	"mediumvioletred":(199,21,133),
 	"white":(255,255,255),
 	"snow":(255,250,250),
 	"honeydew":(240,255,240),
 	"mintcream":(245,255,250),
 	"azure":(240,255,255),
 	"aliceblue":(240,248,255),
 	"ghostwhite":(248,248,255),
 	"whitesmoke":(245,245,245),
 	"seashell":(255,245,238),
 	"beige":(245,245,220),
 	"oldlace":(253,245,230),
 	"floralwhite":(255,250,240),
 	"ivory":(255,255,240),
 	"antiquewhite":(250,235,215),
 	"linen":(250,240,230),
 	"lavenderblush":(255,240,245),
 	"mistyrose":(255,228,225),
 	"gainsboro":(220,220,220),
 	"lightgray":(211,211,211),
 	"silver":(192,192,192),
 	"darkgray":(169,169,169),
 	"gray":(128,128,128),
 	"dimgray":(105,105,105),
 	"lightslategray":(119,136,153),
 	"slategray":(112,128,144),
 	"darkslategray":(47,79,79),
 	"black":(0,0,0),
 	"cornsilk":(255,248,220),
 	"blanchedalmond":(255,235,205),
 	"bisque":(255,228,196),
 	"navajowhite":(255,222,173),
 	"wheat":(245,222,179),
 	"burlywood":(222,184,135),
 	"tan":(210,180,140),
 	"rosybrown":(188,143,143),
 	"sandybrown":(244,164,96),
 	"goldenrod":(218,165,32),
 	"peru":(205,133,63),
 	"chocolate":(210,105,30),
 	"saddlebrown":(139,69,19),
 	"sienna":(160,82,45),
 	"brown":(165,42,42),
 	"maroon":(128,0,0)
}
rng = np.random.default_rng(12345)
lh = [0, len(SCSET) - 1]
rand_color = lambda : STANDARD_COLORS[SCSET[rng.integers(low=lh[0],high=lh[1], size=1)[0]]]