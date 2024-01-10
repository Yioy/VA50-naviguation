
from trajectory_extractor.fish2bird_python import Fish2Bird
import numpy as np
import cv2 as cv

class MultiCamBirdViewInitConfig:
    def __init__(self,
                 in_img_height, in_img_width, Ks, Ds, transforms,
                 x_range, y_range, output_size_y, flip_x=False, flip_y=False):

        self.in_img_height = in_img_height
        self.in_img_width = in_img_width
        self.Ks = Ks
        self.Ds = Ds
        self.transforms = transforms
        self.x_range = x_range
        self.y_range = y_range
        self.output_size_y = output_size_y
        self.flip_x = flip_x
        self.flip_y = flip_y

class MultiCamBirdView:
    
    def __init__(self, config):
        self.config = config

        self.bird_views = []
        for i in range(len(config.Ks)):
            self.bird_views.append(
                Fish2Bird(
                    (config.in_img_height, config.in_img_width),
                    np.array(config.Ks[i]).reshape((3,3)),
                    config.transforms[i],
                    config.Ds[i][0],
                    config.x_range, 
                    config.y_range, 
                    config.output_size_y,
                    flip_x=config.flip_x,
                    flip_y=config.flip_y)
            )
        
    
    def compute_bird_view(self, images):
        """Process the camera images to the bird view image
        :param images: list of camera images
        :return: bird view image
        """

        # Get the bird view images
        bird_views = []
        scale = None
        for i in range(len(images)):
            bv, scale = self.bird_views[i].to_birdeye(images[i])
            bird_views.append(bv)

        # Stitch the bird view images together
        masked_bird_views = self.mask_images(bird_views)
        stitched_image = self.stitch_images(masked_bird_views)

        return stitched_image, scale
    
    def mask_images(self, images):
        # Quick fix : todo improve
        if len(images) == 3:
            images[1][:, int(images[1].shape[1]/2):] = 0
            images[2][:, :int(images[2].shape[1]/2)] = 0
        
        return images
            


    def stitch_images(self, images):
        """Stitch the bird view images together
        :param images: list of bird view images
        :return: stitched image
        """

        stitched_image = images[0]
        for i in range(1, len(images)):
            stitched_image[stitched_image == 0] = images[i][stitched_image == 0]

        return stitched_image
