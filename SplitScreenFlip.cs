using Engine;
using Game;
using HarmonyLib;

namespace SplitScreenFlip
{
	// Loaded by the game for every non-abstract ModLoader subclass in the mod's dll.
	public class SplitScreenFlipLoader : ModLoader
	{
		public override void __ModInitialize()
		{
			new Harmony("com.mrzgaming.splitscreenflip").PatchAll(typeof(SplitScreenFlipLoader).Assembly);
		}
	}

	// Two players, "Double Horizontal" layout (top / bottom):
	// the bottom screen stays as it is, the top screen (player 1) is rotated 180 degrees.
	// Touch input follows the rotation because it goes through the widget transform,
	// the same way the built-in "Opposite" layouts work.
	[HarmonyPatch(typeof(GamesWidget), nameof(GamesWidget.ArrangeOverride))]
	public static class GamesWidgetArrangePatch
	{
		// Same scale the vanilla DoubleHorizontal layout uses.
		private const float Scale = 0.48f;

		public static void Postfix(GamesWidget __instance)
		{
			if (__instance.Children.Count != 2)
			{
				return;
			}
			if (SettingsManager.ScreenLayout2 != ScreenLayout.DoubleHorizontal)
			{
				return;
			}
			if (!(__instance.Children[0] is GameWidget top))
			{
				return;
			}

			// 180 degree rotation (negative X and Y scale), around the widget center.
			top.LayoutTransform = new Matrix(
				-Scale, 0f, 0f, 0f,
				0f, -Scale, 0f, 0f,
				0f, 0f, 1f, 0f,
				0f, 0f, 0f, 1f);

			// The screen is upside down now, so swap the left/right cutout margins.
			ContainerWidget gui = top.GuiWidget;
			float left = gui.MarginLeft;
			gui.MarginLeft = gui.MarginRight;
			gui.MarginRight = left;
		}
	}
}
